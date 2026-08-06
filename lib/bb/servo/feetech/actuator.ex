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
  2. Asks the servo what model it is, and sets its operating mode if it isn't
     already in the configured one
  3. Registers with the controller, receiving the shared ETS table reference
  4. Subscribes to the commands its mode admits

  When a position command is received, the actuator:
  1. Clamps the position to motor limits
  2. Converts to servo position units (0-4095 for 360 degrees)
  3. Writes goal_position and goal_speed to the controller's ETS table
  4. Publishes a `BB.Message.Actuator.BeginMotion` via
     `BB.Actuator.publish_begin_motion/3` (which handles the
     motor → joint-space conversion)

  The controller picks up pending commands on its next loop tick and batches them
  into efficient `sync_write` operations on the serial bus.

  ## Operating modes

  An STS servo does one thing at a time, and `:mode` picks which. It's set once
  at startup and never changed while running: the register lives in EEPROM,
  which has a finite write budget and can only be written with torque off and
  the servo unlocked.

  | `:mode` | commands it accepts |
  | --- | --- |
  | `:position` (default) | `Position`, `Trajectory`, `Effort`, `Hold`, `Stop` |
  | `:velocity` | `Velocity`, `Effort`, `Hold`, `Stop` |

  Anything outside that list is refused by the framework with
  `BB.Error.State.UnsupportedCommand` before it reaches the driver.

  The servo's `:pwm` and `:step` modes aren't offered. PWM is an open-loop duty
  cycle with no feedback, which no `BB` command means; step mode is multi-turn
  positioning, which the joint limits this driver derives its range from don't
  describe.

  ## Commands

  - `Command.Position` — travel to a position. Clamped to the joint's limits,
    and travelling no faster than its velocity limit.
  - `Command.Trajectory` — walk a list of waypoints, one timer per leg.
  - `Command.Velocity` — turn at a rate, clamped to the joint's velocity limit.
  - `Command.Effort` — **a ceiling, not a goal.** These servos have no torque
    goal register; `torque_limit` caps what a move may draw, as a fraction of
    the model's rated stall torque. Setting one won't make the joint move, and
    the torque it eventually produces is approximate — see
    `BB.Servo.Feetech.Model` for how rough. The servo's own overload protection
    can also wind the ceiling down under sustained load.
  - `Command.Stop` — cut torque, leaving the joint passive and free to be
    backdriven. Both `:immediate` and `:decelerate` do the same thing, because
    becoming passive is a single register write with no ramp available.
  - `Command.Hold` — stay under power without driving. A servo already doing
    that needs nothing; after a `Stop` it re-applies torque where the joint has
    come to rest.

  A motion command sent to a joint left passive by `Stop` re-applies torque on
  the way past, so callers don't have to pair the two.

  `Velocity` and `Effort` carry a `duration`. When it runs out, `:expiry_action`
  decides whether the joint goes passive (`:stop`, the default) or stays under
  power without driving (`:hold`) — the same choice the controller's
  `:disarm_action` makes for the whole bus.

  Beware that a passive joint under load will move, and `Stop` does not wait for
  it to settle. `Hold` and `Position` both re-apply torque from the servo's own
  present position, so neither snaps back to a pre-`Stop` goal, but a joint that
  has sagged will still be somewhere the caller may not expect.

  None of this is the safety path: making the hardware safe is `disarm/1`, which
  is robot-wide and leaves the robot unable to move until it is armed again.

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
  import BB.Unit.Option

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
      ],
      mode: [
        type: {:in, [:position, :velocity]},
        doc: """
        The servo's operating mode, which decides what it can be commanded to
        do. Set once at startup and never changed while running, because the
        register lives in EEPROM.
        """,
        default: :position
      ],
      stall_torque: [
        type: unit_type(compatible: :newton_meter),
        doc: """
        The servo's rated stall torque, used to scale `Command.Effort` into a
        `torque_limit` ceiling. Defaults to the figure in
        `BB.Servo.Feetech.Model` for the model the servo reports. Set it
        yourself if the servo isn't recognised, or if it is an STS3215 variant
        other than the 7.4V 1:345 — they all report the same model number.
        """,
        required: false
      ],
      expiry_action: [
        type: {:in, [:hold, :stop]},
        doc: """
        What to do when a velocity or effort command's `duration` runs out:
        `:stop` to go passive, or `:hold` to stay under power without driving.
        A joint that carries a load wants `:hold`; one that should be
        backdrivable when nothing is asking it to move wants `:stop`.
        """,
        default: :stop
      ]
    ]

  alias BB.Error.Invalid.Feetech.StallTorque, as: StallTorqueError
  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess
  alias BB.Robot.Units
  alias BB.Servo.Feetech.Model

  defmodule State do
    @moduledoc false
    defstruct [
      :bb,
      :controller,
      :current_motor_angle,
      :expiry_timer,
      :joint_name,
      :mode,
      :motor_profile,
      :name,
      :servo_id,
      :servo_table,
      :stall_torque,
      :trajectory,
      :trajectory_timer,
      expiry_action: :stop,
      position_deadband: 2
    ]
  end

  @position_resolution 4096
  @position_center 2048

  # The speed register counts encoder steps per second, so it tops out one step
  # short of a revolution per second — the same ceiling the position register
  # has. A joint may be declared faster than that: the SO-101's
  # `360 degree_per_second` lands on exactly 4096, one past the end. Writing it
  # would put an out-of-range value on the bus, so speeds are clamped to the
  # register as well as to the joint.
  @max_motor_speed (@position_resolution - 1) * 2 * :math.pi() / @position_resolution

  # ETS tuple field indices for command writes
  @ets_idx_pending_writes 10
  @ets_idx_pending_limit 11

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
         {:ok, state} <- resolve_stall_torque(state),
         :ok <- configure_mode(state),
         {:ok, servo_table} <- register_servo(state) do
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
        expiry_action: Map.get(opts, :expiry_action, :stop),
        joint_name: joint_name,
        mode: Map.get(opts, :mode, :position),
        motor_profile: motor_profile,
        name: name,
        position_deadband: Map.get(opts, :position_deadband, 2),
        servo_id: opts.servo_id,
        stall_torque: newton_metres(Map.get(opts, :stall_torque))
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

  defp disable_torque(state), do: write_param(state, :torque_enable, false)

  # Every mode accepts `Effort`, so every servo needs a rating to scale it
  # against — either one the caller supplied or one this model is known by.
  # Refusing to start beats accepting effort commands and guessing at them.
  defp resolve_stall_torque(%{stall_torque: stall_torque} = state)
       when is_number(stall_torque),
       do: {:ok, state}

  defp resolve_stall_torque(state) do
    with {:ok, model_number} <- read_param(state, :model_number) do
      case Model.fetch(model_number) do
        {:ok, model} ->
          {:ok, %{state | stall_torque: model.stall_torque}}

        :error ->
          {:error, %StallTorqueError{servo_id: state.servo_id, model_number: model_number}}
      end
    end
  end

  # Torque is already off at this point, which `mode` needs — it lives in
  # EEPROM, behind the `lock` register. Writing it unconditionally would spend
  # an EEPROM write on every restart, so only write when the servo isn't
  # already in the right mode, and put the lock back the way it was found.
  defp configure_mode(state) do
    with {:ok, current_mode} <- read_param(state, :mode) do
      if current_mode == state.mode do
        :ok
      else
        write_mode(state)
      end
    end
  end

  defp write_mode(state) do
    with :ok <- write_param(state, :lock, false),
         :ok <- write_param(state, :mode, state.mode) do
      write_param(state, :lock, true)
    end
  end

  defp read_param(state, param) do
    BBProcess.call(state.bb.robot, state.controller, {:read, state.servo_id, param})
  end

  defp write_param(state, param, value) do
    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:write, state.servo_id, param, value}
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

  # What the servo can be asked for depends on the mode it's configured in.
  # Declaring it means the framework refuses everything else with a structured
  # error, rather than the driver accepting a command and doing nothing with it.
  #
  # `Stop`, `Hold` and `Effort` are in both modes: cutting torque works whatever
  # the servo is doing, standing still is meaningful either way, and
  # `torque_limit` caps the current a move may draw regardless of what started
  # the move.
  @impl BB.Actuator
  def command_payloads(opts) do
    opts
    |> Keyword.get(:mode, :position)
    |> mode_payloads()
    |> Enum.sort()
  end

  defp mode_payloads(:position),
    do: [Command.Effort, Command.Hold, Command.Position, Command.Stop, Command.Trajectory]

  defp mode_payloads(:velocity),
    do: [Command.Effort, Command.Hold, Command.Stop, Command.Velocity]

  @impl BB.Actuator
  def handle_command(%Message{payload: %Command.Position{} = cmd}, state) do
    do_set_position(cmd, cancel_expiry(state))
  end

  def handle_command(%Message{payload: %Command.Trajectory{} = cmd}, state) do
    start_trajectory(cmd, cancel_expiry(state))
  end

  # `Stop` means cease travelling and become passive — the counterpart to
  # `Hold`, which maintains position. Cutting torque is the only way an STS
  # servo becomes passive, so both stop modes arrive here: there is no register
  # write that decelerates *into* being passive, and reporting `:decelerate` as
  # unsupported would refuse a stop rather than perform one.
  #
  # The pending goal goes first, so a resume can't act on a target the caller
  # has abandoned, and so it isn't written to a servo on its way to limp.
  #
  # This is not the safety path: making hardware safe is `disarm/1`.
  def handle_command(%Message{payload: %Command.Stop{}}, state) do
    state = cancel_trajectory(cancel_expiry(state))
    reply(state, stop_driving(state))
  end

  # These servos hold whatever they were last told whenever torque is on, so
  # `Hold` has nothing to do in the common case. It is not a no-op after a
  # `Stop`: torque has to come back on, and at the position the joint has come
  # to rest at rather than the goal it was chasing when it went limp. In
  # velocity mode there is no position to return to — holding is commanding a
  # standstill, which a powered servo resists being moved from.
  def handle_command(%Message{payload: %Command.Hold{}}, state) do
    state
    |> cancel_expiry()
    |> cancel_trajectory()
    |> do_hold()
  end

  def handle_command(%Message{payload: %Command.Velocity{} = cmd}, state) do
    state
    |> schedule_expiry(cmd.duration)
    |> apply_and_reply([{:goal_speed, clamp_speed(cmd.velocity, state)}])
  end

  # There is no torque goal on these servos. `torque_limit` is a ceiling on what
  # a move may draw, expressed as a fraction of the model's rated stall torque —
  # which is how you get a gripper that squeezes to a limit rather than to a
  # position. Setting it will not, on its own, make the joint move.
  def handle_command(%Message{payload: %Command.Effort{} = cmd}, state) do
    state = schedule_expiry(state, cmd.duration)
    fraction = effort_to_fraction(cmd.effort, state)

    # A ceiling outlives the move it was set for, so it gets its own slot rather
    # than sharing the one a motion command replaces wholesale. On the resume
    # path it travels with the goal, because torque has to come back on and the
    # ceiling must be in place before the servo starts pulling against it.
    if torque_enabled?(state) do
      write_torque_limit(state, fraction)
      {:noreply, state}
    else
      reply(state, resume(state, [{:torque_limit, fraction}]))
    end
  end

  @impl BB.Actuator
  def handle_info(:trajectory_next, state), do: advance_trajectory(state)

  # `duration` on a velocity or effort command is how long to drive for. What
  # happens when it runs out is the joint's business rather than the caller's,
  # so it comes from `:expiry_action` — the same choice `:disarm_action` makes
  # for the bus, one joint at a time.
  def handle_info({:expire_command, timer}, %{expiry_timer: timer} = state) do
    state = %{state | expiry_timer: nil}

    case state.expiry_action do
      :stop -> reply(state, stop_driving(state))
      :hold -> do_hold(state)
    end
  end

  def handle_info(_message, state), do: {:noreply, state}

  # A powered servo in position mode is already holding whatever it was last
  # told, so there is nothing to write. The `:present_position` sentinel is only
  # meaningful on the resume path, where the controller resolves it against the
  # bus — writing it as a goal would be a write to a read-only register.
  defp do_hold(state) do
    cond do
      not torque_enabled?(state) -> reply(state, resume(state, hold_write(state)))
      state.mode == :velocity -> apply_and_reply(state, hold_write(state))
      true -> {:noreply, state}
    end
  end

  # --- Applying writes ---

  # The fast path leaves the write for the controller's next tick to batch onto
  # the bus. A servo left passive by a `Stop` can't be commanded that way — the
  # value would land in a register the servo isn't acting on — so it takes the
  # controller's ordered resume instead, going to what was just asked for rather
  # than to whatever it was chasing before.
  defp apply_write(state, pending_writes) do
    if torque_enabled?(state) do
      write_servo_command(state, pending_writes)
      :ok
    else
      resume(state, pending_writes)
    end
  end

  defp apply_and_reply(state, pending_writes),
    do: reply(state, apply_write(state, pending_writes))

  defp reply(state, :ok), do: {:noreply, state}
  defp reply(state, {:error, reason}), do: {:stop, reason, state}

  # Re-enabling torque without a fresh goal makes the servo lunge for whatever
  # it was chasing before, so a resume always carries one. An `Effort` ceiling
  # on its own doesn't move anything, so it gets the standstill goal that `Hold`
  # would have used.
  @motion_params [:goal_position, :goal_speed, :present_position]

  defp resume(state, pending_writes) do
    writes =
      if Enum.any?(pending_writes, fn {param, _} -> param in @motion_params end) do
        pending_writes
      else
        pending_writes ++ hold_write(state)
      end

    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:resume_servo, state.servo_id, writes}
    )
  end

  defp stop_driving(state) do
    clear_pending_writes(state)
    disable_torque(state)
  end

  # Holding means "stop driving but stay under power", which is a different
  # register in each mode. `:present_position` is a sentinel the controller
  # resolves against the bus, because only it knows where the joint has come to
  # rest.
  defp hold_write(%{mode: :velocity}), do: [{:goal_speed, 0.0}]
  defp hold_write(_state), do: [{:present_position, nil}]

  defp schedule_expiry(state, nil), do: cancel_expiry(state)

  defp schedule_expiry(state, duration) do
    state = cancel_expiry(state)
    timer = make_ref()
    Process.send_after(self(), {:expire_command, timer}, duration)
    %{state | expiry_timer: timer}
  end

  # The reference, rather than the timer, is what `handle_info/2` matches on, so
  # a message already in the mailbox when a new command arrives is ignored
  # rather than cutting the new command short.
  defp cancel_expiry(state), do: %{state | expiry_timer: nil}

  defp torque_enabled?(state) do
    case :ets.lookup(state.servo_table, state.servo_id) do
      [{_, _, _, _, _, _, _, _, _, _, _, torque_enabled}] -> torque_enabled == true
      [] -> false
    end
  rescue
    # The controller owns the table; if it has gone, so has the bus.
    ArgumentError -> false
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

    writes = [{:goal_speed, goal_speed}, {:goal_position, goal_position}]

    case apply_write(state, writes) do
      :ok ->
        publish_begin_motion(cmd, clamped_motor_angle, state)
        {:noreply, %{state | current_motor_angle: clamped_motor_angle}}

      {:error, reason} ->
        {:stop, reason, state}
    end
  end

  defp publish_begin_motion(cmd, clamped_motor_angle, state) do
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
  end

  # A `goal_speed` of 0 means "as fast as the servo can", which ignores the
  # joint's declared velocity limit and arrives sooner than the `BeginMotion`
  # estimate — computed from that limit — predicts. Falling back to the limit
  # makes it real. Anything the caller asks for is clamped to it too.
  defp compute_goal_speed(%Command.Position{velocity: velocity}, _clamped_angle, state)
       when is_number(velocity),
       do: clamp_speed(abs(velocity), state)

  defp compute_goal_speed(%Command.Position{duration: duration}, clamped_motor_angle, state)
       when is_integer(duration) and duration > 0 do
    travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
    clamp_speed(travel_distance / (duration / 1000), state)
  end

  defp compute_goal_speed(_cmd, _clamped_angle, state),
    do: clamp_speed(state.motor_profile.motor_velocity_limit, state)

  defp clear_pending_writes(state) do
    update_row(state, [{@ets_idx_pending_writes, nil}, {@ets_idx_pending_limit, nil}])
  end

  defp write_servo_command(state, pending_writes) do
    update_row(state, [{@ets_idx_pending_writes, pending_writes}])
  end

  defp write_torque_limit(state, fraction) do
    update_row(state, [{@ets_idx_pending_limit, fraction}])
  end

  defp update_row(state, elements) do
    :ets.update_element(state.servo_table, state.servo_id, elements)
  rescue
    # The controller owns the table; if it has gone, so has the bus.
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

  defp advance_trajectory(%{trajectory: nil} = state), do: {:noreply, state}

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
    {:noreply, %{state | trajectory: nil, trajectory_timer: nil}}
  end

  defp execute_and_schedule(trajectory, state) do
    waypoint = Enum.at(trajectory.waypoints, trajectory.index)
    clamped_motor_angle = clamp_motor_angle(waypoint[:position], state.motor_profile)
    goal_speed = waypoint_speed(waypoint, state)
    goal_position = motor_angle_to_position(clamped_motor_angle)

    case apply_write(state, [{:goal_speed, goal_speed}, {:goal_position, goal_position}]) do
      :ok ->
        state = %{state | current_motor_angle: clamped_motor_angle}
        {:noreply, schedule_next_waypoint(trajectory, waypoint, state)}

      {:error, reason} ->
        {:stop, reason, state}
    end
  end

  # A waypoint with no velocity of its own travels at the joint's limit rather
  # than at the servo's maximum, the same as a bare position command.
  defp waypoint_speed(waypoint, state) do
    velocity = waypoint[:velocity]

    if is_number(velocity) and velocity != 0 do
      clamp_speed(abs(velocity), state)
    else
      clamp_speed(state.motor_profile.motor_velocity_limit, state)
    end
  end

  defp schedule_next_waypoint(trajectory, waypoint, state) do
    next_index = trajectory.index + 1

    delay =
      if next_index < length(trajectory.waypoints) do
        next_waypoint = Enum.at(trajectory.waypoints, next_index)
        next_waypoint[:time_from_start] - waypoint[:time_from_start]
      else
        0
      end

    %{state | trajectory_timer: Process.send_after(self(), :trajectory_next, delay)}
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

  defp newton_metres(nil), do: nil

  defp newton_metres(unit) do
    unit
    |> Localize.Unit.convert!(BB.Unit.unit_name(:newton_meter))
    |> Units.extract_float()
  end

  # Velocity limits are magnitudes; the joint may travel either way.
  defp clamp_speed(speed, state) do
    limit = min(state.motor_profile.motor_velocity_limit, @max_motor_speed)

    speed
    |> max(-limit)
    |> min(limit)
  end

  # `torque_limit` takes a fraction of what the servo can produce, so the effort
  # is divided by the model's rated stall torque. A ceiling has no direction,
  # and asking for more than the servo has is the same as asking for all of it.
  defp effort_to_fraction(effort, state) do
    effort
    |> abs()
    |> min(effort_limit(state))
    |> Kernel./(state.stall_torque)
    |> min(1.0)
  end

  defp effort_limit(%{motor_profile: %{motor_effort_limit: limit}}) when is_number(limit),
    do: limit

  defp effort_limit(state), do: state.stall_torque

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
