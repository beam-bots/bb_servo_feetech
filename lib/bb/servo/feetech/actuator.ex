# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Actuator do
  @moduledoc """
  An actuator that uses a Feetech controller to drive a serial bus servo.

  This actuator derives its configuration from the joint constraints defined in the robot:
  - Position limits from `joint.limits.lower` and `joint.limits.upper`
  - Velocity limit from `joint.limits.velocity`
  - Position range maps to the servo's goal_position register

  When initialised, the actuator:
  1. Sets the goal position to joint center (servo won't move until armed)
  2. Registers the servo with the controller for position feedback and torque management

  When a position command is received, the actuator:
  1. Clamps the position to joint limits
  2. Converts to servo position units (0-4095 for 360 degrees)
  3. Sends goal_position command to the Feetech controller
  4. Publishes a `BB.Message.Actuator.BeginMotion` message

  Position feedback and torque management are handled by the controller. Servos
  remain unpowered until the robot is armed, then move to their goal positions.

  ## Example DSL Usage

      controller :feetech, {BB.Servo.Feetech.Controller,
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000
      }

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
      reverse?: [
        type: :boolean,
        doc: "Reverse the servo rotation direction?",
        default: false
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
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  @position_resolution 4096
  @position_center 2048

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
         :ok <- set_initial_position(state) do
      :ok =
        BBProcess.call(
          state.bb.robot,
          state.controller,
          {:register_servo, state.servo_id, state.joint_name, state.center_angle,
           state.position_deadband, state.reverse?}
        )

      BB.subscribe(state.bb.robot, [:actuator, state.joint_name, state.name])

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    robot = opts.bb.robot.robot()

    reverse? = Map.get(opts, :reverse?, false)
    position_deadband = Map.get(opts, :position_deadband, 2)

    with {:ok, joint} <- fetch_joint(robot, joint_name),
         {:ok, limits} <- validate_joint_limits(joint, joint_name) do
      lower_limit = limits.lower
      upper_limit = limits.upper
      range = upper_limit - lower_limit
      center_angle = (lower_limit + upper_limit) / 2
      velocity_limit = limits.velocity

      state = %{
        bb: opts.bb,
        servo_id: opts.servo_id,
        controller: opts.controller,
        reverse?: reverse?,
        position_deadband: position_deadband,
        lower_limit: lower_limit,
        upper_limit: upper_limit,
        center_angle: center_angle,
        range: range,
        velocity_limit: velocity_limit,
        current_angle: center_angle,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp fetch_joint(robot, joint_name) do
    case BB.Robot.get_joint(robot, joint_name) do
      nil ->
        {:error,
         %JointConfigError{joint: joint_name, field: nil, message: "Joint not found in robot"}}

      joint ->
        {:ok, joint}
    end
  end

  defp validate_joint_limits(%{type: :continuous}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :type,
       value: :continuous,
       expected: [:revolute, :prismatic],
       message: "Continuous joints require position limits for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :limits,
       value: nil,
       message: "Joint must have limits defined for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: %{lower: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have lower limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: %{upper: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have upper limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: limits}, _joint_name) do
    {:ok, limits}
  end

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

  defp set_initial_position(state) do
    position = angle_to_position(state.center_angle, state)

    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:write_raw, state.servo_id, :goal_position, position}
         ) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, _} = error -> error
    end
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, _state} = do_set_position(cmd.position, cmd.command_id, state)
    else
      {:noreply, state}
    end
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    if BB.Safety.armed?(state.bb.robot) do
      do_set_position(cmd.position, cmd.command_id, state)
    else
      {:noreply, state}
    end
  end

  @impl BB.Actuator
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    if BB.Safety.armed?(state.bb.robot) do
      {:noreply, new_state} = do_set_position(cmd.position, cmd.command_id, state)
      {:reply, {:ok, :accepted}, new_state}
    else
      {:reply, {:error, :not_armed}, state}
    end
  end

  defp do_set_position(angle, command_id, state) when is_integer(angle),
    do: do_set_position(angle * 1.0, command_id, state)

  defp do_set_position(angle, command_id, state) do
    clamped_angle = clamp_angle(angle, state)
    new_position = angle_to_position(clamped_angle, state)

    :ok =
      BBProcess.cast(
        state.bb.robot,
        state.controller,
        {:write_raw, state.servo_id, :goal_position, new_position}
      )

    travel_distance = abs(state.current_angle - clamped_angle)
    travel_time_ms = round(travel_distance / state.velocity_limit * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts =
      [
        initial_position: state.current_angle,
        target_position: clamped_angle,
        expected_arrival: expected_arrival,
        command_type: :position
      ]
      |> maybe_add_opt(:command_id, command_id)

    message = Message.new!(BeginMotion, state.joint_name, message_opts)

    BB.publish(state.bb.robot, [:actuator | state.bb.path], message)

    {:noreply, %{state | current_angle: clamped_angle}}
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_angle(angle, state) do
    angle
    |> max(state.lower_limit)
    |> min(state.upper_limit)
  end

  defp angle_to_position(angle_rad, state) do
    offset_rad = angle_rad - state.center_angle
    offset_units = offset_rad / (2 * :math.pi()) * @position_resolution

    position =
      if state.reverse? do
        @position_center - offset_units
      else
        @position_center + offset_units
      end

    round(position)
    |> max(0)
    |> min(@position_resolution - 1)
  end
end
