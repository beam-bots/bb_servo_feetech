# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Actuator do
  @moduledoc """
  An actuator that uses a Feetech controller to drive a serial bus servo.

  Configuration is derived from the joint's `motor_profile` injected by
  `BB.Actuator.Server`:

  - Position limits from `motor_profile.motor_lower` / `motor_upper`
  - Velocity limit from `motor_profile.motor_velocity_limit`
  - Position range maps to the servo's goal_position register

  When initialised, the actuator:
  1. Disables torque on the servo
  2. Registers with the controller, receiving the shared ETS table reference
  3. Subscribes to position commands

  When a position command is received, the actuator:
  1. Clamps the position to motor limits
  2. Converts to servo position units (0-4095 for 360 degrees)
  3. Writes goal_position and goal_speed to the controller's ETS table
  4. Publishes a `BB.Message.Actuator.BeginMotion` via
     `BB.Actuator.publish_begin_motion/3` (which handles the
     motor → joint-space conversion)

  The controller picks up pending commands on its next loop tick and batches them
  into efficient `sync_write` operations on the serial bus.

  ## Example DSL Usage

      controllers do
        controller :feetech, {BB.Servo.Feetech.Controller,
          port: "/dev/ttyUSB0",
          baud_rate: 1_000_000
        }
      end

      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Feetech.Actuator, servo_id: 1, controller: :feetech}
      end
  """
  use BB.Actuator,
    options_schema: [
      servo_id: [
        type: {:in, 1..253},
        doc: "The Feetech servo ID (1-253)",
        required: true
      ],
      controller: [
        type: :atom,
        doc: "Name of the Feetech controller in the robot's registry",
        required: true
      ],
      position_deadband: [
        type: :non_neg_integer,
        doc:
          "Minimum position change (raw units) to trigger feedback publish. Filters servo noise.",
        default: 2
      ]
    ]

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  defmodule State do
    @moduledoc false
    defstruct [
      :bb,
      :controller,
      :current_motor_angle,
      :joint_name,
      :motor_profile,
      :name,
      :servo_id,
      :servo_table,
      :trajectory,
      :trajectory_timer,
      position_deadband: 2
    ]
  end

  @position_resolution 4096
  @position_center 2048

  # ETS tuple field indices for command writes
  @ets_idx_goal_position 10
  @ets_idx_goal_speed 11

  @doc """
  Safety disarm callback.

  Returns :ok because torque management is handled by the controller.
  The controller receives all registered servo IDs and disables torque
  for all of them in a single sync_write operation, which is more
  efficient for bus-based protocols.
  """
  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    with {:ok, state} <- build_state(opts),
         :ok <- disable_torque(state),
         {:ok, servo_table} <- register_servo(state) do
      BB.subscribe(state.bb.robot, [:actuator, state.joint_name, state.name])

      {:ok, %{state | servo_table: servo_table}}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    motor_profile = Keyword.fetch!(new_opts, :motor_profile)

    {:ok,
     %{
       state
       | motor_profile: motor_profile,
         current_motor_angle: clamp_motor_angle(state.current_motor_angle, motor_profile)
     }}
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    motor_profile = opts.motor_profile

    with :ok <- validate_motor_profile(motor_profile, joint_name) do
      state = %State{
        bb: opts.bb,
        controller: opts.controller,
        current_motor_angle: motor_profile.motor_initial_position,
        joint_name: joint_name,
        motor_profile: motor_profile,
        name: name,
        position_deadband: Map.get(opts, :position_deadband, 2),
        servo_id: opts.servo_id
      }

      {:ok, state}
    end
  end

  defp validate_motor_profile(%{motor_lower: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have a lower limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_upper: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have an upper limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_velocity_limit: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :velocity,
       value: nil,
       message: "Joint must have a velocity limit defined for servo control"
     }}
  end

  defp validate_motor_profile(_profile, _joint_name), do: :ok

  defp disable_torque(state) do
    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:write, state.servo_id, :torque_enable, false}
         ) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, _} = error -> error
    end
  end

  defp register_servo(state) do
    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:register_servo, state.servo_id, state.bb.path, state.position_deadband}
    )
  end

  # --- Command handling ---

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, _state} = do_set_position(cmd, state)
    else
      {:noreply, state}
    end
  end

  def handle_info({:bb, _path, %Message{payload: %Command.Trajectory{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, start_trajectory(cmd, state)}
    else
      {:noreply, state}
    end
  end

  def handle_info(:trajectory_next, state) do
    {:noreply, advance_trajectory(state)}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      do_set_position(cmd, state)
    else
      {:noreply, state}
    end
  end

  def handle_cast({:command, %Message{payload: %Command.Trajectory{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, start_trajectory(cmd, state)}
    else
      {:noreply, state}
    end
  end

  @impl BB.Actuator
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, new_state} = do_set_position(cmd, state)
      {:reply, {:ok, :accepted}, new_state}
    else
      {:reply, {:error, :not_armed}, state}
    end
  end

  def handle_call({:command, %Message{payload: %Command.Trajectory{} = cmd}}, _from, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:reply, {:ok, :accepted}, start_trajectory(cmd, state)}
    else
      {:reply, {:error, :not_armed}, state}
    end
  end

  # --- Position commands ---

  defp do_set_position(%Command.Position{position: angle} = cmd, state)
       when is_integer(angle),
       do: do_set_position(%{cmd | position: angle * 1.0}, state)

  defp do_set_position(%Command.Position{} = cmd, state) do
    state = cancel_trajectory(state)
    clamped_motor_angle = clamp_motor_angle(cmd.position, state.motor_profile)

    goal_speed = compute_goal_speed(cmd, clamped_motor_angle, state)
    goal_position = motor_angle_to_position(clamped_motor_angle)

    write_servo_command(state, goal_position, goal_speed)

    travel_time_ms = estimate_travel_time_ms(cmd, clamped_motor_angle, state)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts =
      [
        initial_position: state.current_motor_angle,
        target_position: clamped_motor_angle,
        expected_arrival: expected_arrival,
        command_type: :position
      ]
      |> maybe_add_opt(:command_id, cmd.command_id)

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)

    {:noreply, %{state | current_motor_angle: clamped_motor_angle}}
  end

  defp compute_goal_speed(%Command.Position{velocity: velocity}, _clamped_angle, _state)
       when is_number(velocity),
       do: abs(velocity)

  defp compute_goal_speed(%Command.Position{duration: duration}, clamped_motor_angle, state)
       when is_integer(duration) and duration > 0 do
    travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
    travel_distance / (duration / 1000)
  end

  defp compute_goal_speed(_cmd, _clamped_angle, _state), do: 0

  defp write_servo_command(state, goal_position, goal_speed) do
    :ets.update_element(state.servo_table, state.servo_id, [
      {@ets_idx_goal_position, goal_position},
      {@ets_idx_goal_speed, goal_speed}
    ])
  rescue
    ArgumentError -> false
  end

  defp estimate_travel_time_ms(%Command.Position{velocity: velocity}, clamped_motor_angle, state)
       when is_number(velocity) and velocity > 0 do
    travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
    round(travel_distance / abs(velocity) * 1000)
  end

  defp estimate_travel_time_ms(%Command.Position{duration: duration}, _clamped_angle, _state)
       when is_integer(duration) and duration > 0 do
    duration
  end

  defp estimate_travel_time_ms(_cmd, clamped_motor_angle, state) do
    travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
    round(travel_distance / state.motor_profile.motor_velocity_limit * 1000)
  end

  # --- Trajectory commands ---

  defp start_trajectory(%Command.Trajectory{waypoints: waypoints} = cmd, state) do
    state = cancel_trajectory(state)

    trajectory = %{
      waypoints: waypoints,
      index: 0,
      repeat: cmd.repeat || 1,
      command_id: cmd.command_id,
      started_at: System.monotonic_time(:millisecond)
    }

    last_time = List.last(waypoints)[:time_from_start]
    expected_arrival = System.monotonic_time(:millisecond) + last_time

    message_opts =
      [
        initial_position: state.current_motor_angle,
        target_position: hd(waypoints)[:position],
        expected_arrival: expected_arrival,
        command_type: :trajectory
      ]
      |> maybe_add_opt(:command_id, cmd.command_id)

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)

    execute_and_schedule(trajectory, %{state | trajectory: trajectory})
  end

  defp advance_trajectory(%{trajectory: nil} = state), do: state

  defp advance_trajectory(%{trajectory: trajectory} = state) do
    next_index = trajectory.index + 1

    if next_index >= length(trajectory.waypoints) do
      handle_trajectory_end(trajectory, state)
    else
      trajectory = %{trajectory | index: next_index}
      execute_and_schedule(trajectory, %{state | trajectory: trajectory})
    end
  end

  defp handle_trajectory_end(%{repeat: :forever} = trajectory, state) do
    trajectory = %{trajectory | index: 0, started_at: System.monotonic_time(:millisecond)}
    execute_and_schedule(trajectory, %{state | trajectory: trajectory})
  end

  defp handle_trajectory_end(%{repeat: n} = trajectory, state) when n > 1 do
    trajectory = %{
      trajectory
      | index: 0,
        repeat: n - 1,
        started_at: System.monotonic_time(:millisecond)
    }

    execute_and_schedule(trajectory, %{state | trajectory: trajectory})
  end

  defp handle_trajectory_end(_trajectory, state) do
    %{state | trajectory: nil, trajectory_timer: nil}
  end

  defp execute_and_schedule(trajectory, state) do
    waypoint = Enum.at(trajectory.waypoints, trajectory.index)
    velocity = abs(waypoint[:velocity] || 0)
    clamped_motor_angle = clamp_motor_angle(waypoint[:position], state.motor_profile)

    goal_position = motor_angle_to_position(clamped_motor_angle)
    write_servo_command(state, goal_position, velocity)

    state = %{state | current_motor_angle: clamped_motor_angle}

    next_index = trajectory.index + 1

    if next_index < length(trajectory.waypoints) do
      next_waypoint = Enum.at(trajectory.waypoints, next_index)
      delay = next_waypoint[:time_from_start] - waypoint[:time_from_start]
      timer = Process.send_after(self(), :trajectory_next, delay)
      %{state | trajectory_timer: timer}
    else
      timer = Process.send_after(self(), :trajectory_next, 0)
      %{state | trajectory_timer: timer}
    end
  end

  defp cancel_trajectory(%{trajectory_timer: nil} = state), do: state

  defp cancel_trajectory(%{trajectory_timer: timer} = state) do
    Process.cancel_timer(timer)
    %{state | trajectory: nil, trajectory_timer: nil}
  end

  # --- Helpers ---

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_motor_angle(motor_angle, %{motor_lower: lower, motor_upper: upper}) do
    motor_angle
    |> max(lower)
    |> min(upper)
  end

  # The Feetech servo encoder is 0..@position_resolution-1, mapped linearly to
  # one full motor rotation. Encoder centre (@position_center) corresponds to
  # motor zero. Negative motor angles map to below centre, positive to above.
  defp motor_angle_to_position(motor_angle_rad) do
    offset_units = motor_angle_rad / (2 * :math.pi()) * @position_resolution

    round(@position_center + offset_units)
    |> max(0)
    |> min(@position_resolution - 1)
  end
end
