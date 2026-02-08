# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Controller do
  @moduledoc """
  A controller that manages a Feetech servo bus.

  This controller wraps the `Feetech` GenServer and provides an interface
  for actuators to communicate with servos via the serial bus. Multiple
  actuators can share a single controller, with each actuator controlling
  a different servo ID (1-253).

  ## Position Feedback

  The controller handles position feedback for all registered servos using
  efficient bulk reads (`sync_read`). When actuators register with the
  controller, they provide their joint mapping information. The controller
  then polls all registered servos at a configurable interval and publishes
  `JointState` messages.

  This eliminates the need for separate sensor GenServers and prevents
  bus contention from multiple concurrent read requests.

  ## Configuration

  The controller is typically defined in the robot DSL:

      controller :feetech, {BB.Servo.Feetech.Controller,
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000,
        control_table: Feetech.ControlTable.STS3215,
        poll_interval_ms: 50
      }

  ## Options

  - `:port` - (required) The serial port path, e.g., `"/dev/ttyUSB0"`
  - `:baud_rate` - Baud rate in bps (default: 1_000_000)
  - `:control_table` - The servo control table to use (default: `Feetech.ControlTable.STS3215`)
  - `:poll_interval_ms` - Position feedback interval in ms (default: 50, i.e. 20Hz)
  - `:status_poll_interval_ms` - Status polling interval in ms (default: 1000, set to 0 to disable)
  - `:disarm_action` - Action to take when robot is disarmed (default: `:disable_torque`)
    - `:disable_torque` - Disable torque on all servos (safe default)
    - `:hold` - Keep torque enabled (servos hold position)

  ## Status Polling

  When enabled, the controller periodically reads status registers (temperature, voltage,
  load, hardware errors) from all registered servos and publishes `ServoStatus` messages
  to the sensor path.

  ## Safety

  This controller implements the `BB.Safety` behaviour. When the robot is disarmed
  or crashes, torque is disabled on all known servo IDs using sync_write for speed.
  """
  use BB.Controller,
    options_schema: [
      port: [
        type: :string,
        doc: "The serial port path (e.g., \"/dev/ttyUSB0\")",
        required: true
      ],
      baud_rate: [
        type: :pos_integer,
        doc: "Baud rate in bps",
        default: 1_000_000
      ],
      control_table: [
        type: {:behaviour, Feetech.ControlTable},
        doc: "The servo control table to use",
        default: Feetech.ControlTable.STS3215
      ],
      poll_interval_ms: [
        type: :pos_integer,
        doc: "Position feedback polling interval in milliseconds",
        default: 50
      ],
      status_poll_interval_ms: [
        type: :non_neg_integer,
        doc: "Status polling interval in milliseconds (0 to disable)",
        default: 1000
      ],
      disarm_action: [
        type: {:in, [:disable_torque, :hold]},
        doc: "Action to take when robot is disarmed",
        default: :disable_torque
      ]
    ]

  require Logger

  alias BB.Diagnostic
  alias BB.Error.Protocol.Feetech.HardwareAlert
  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Servo.Feetech.Message.ServoStatus
  alias BB.StateMachine.Transition

  @position_resolution 4096

  # Diagnostic thresholds (Feetech servos typically run on 6-7.4V)
  @temp_warning_threshold 55.0
  @temp_error_threshold 70.0
  @voltage_low_threshold 5.5
  @voltage_high_threshold 8.0

  @doc """
  Handle disarm based on the configured `disarm_action`.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  By default, disables torque on all registered servo IDs.
  """
  @impl BB.Controller
  def disarm(opts) do
    disarm_action = Keyword.get(opts, :disarm_action, :disable_torque)
    do_disarm(disarm_action, opts)
  end

  defp do_disarm(:hold, _opts), do: :ok

  defp do_disarm(:disable_torque, opts) do
    feetech = Keyword.fetch!(opts, :feetech)
    servo_ids = Keyword.get(opts, :servo_ids, [])

    try do
      if servo_ids != [] do
        values = Enum.map(servo_ids, fn id -> {id, false} end)
        # Write to both torque_enable AND lock (as lerobot does)
        Feetech.sync_write(feetech, :torque_enable, values)
        Feetech.sync_write(feetech, :lock, values)
      end

      :ok
    catch
      :exit, _ -> :ok
    end
  end

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    control_table = Keyword.get(opts, :control_table, Feetech.ControlTable.STS3215)
    poll_interval_ms = Keyword.get(opts, :poll_interval_ms, 50)
    status_poll_interval_ms = Keyword.get(opts, :status_poll_interval_ms, 1000)
    disarm_action = Keyword.get(opts, :disarm_action, :disable_torque)

    case start_feetech(opts) do
      {:ok, feetech} ->
        state = %{
          bb: bb,
          feetech: feetech,
          control_table: control_table,
          name: List.last(bb.path),
          poll_interval_ms: poll_interval_ms,
          status_poll_interval_ms: status_poll_interval_ms,
          disarm_action: disarm_action,
          servo_registry: %{},
          last_status: %{}
        }

        BB.Safety.register(__MODULE__,
          robot: state.bb.robot,
          path: state.bb.path,
          opts: [feetech: state.feetech, servo_ids: [], disarm_action: state.disarm_action]
        )

        BB.subscribe(state.bb.robot, [:state_machine])

        Process.send_after(self(), :start_polling, 100)

        {:ok, state}

      {:error, reason} ->
        {:stop, reason}
    end
  end

  defp start_feetech(opts) do
    Feetech.start_link(
      port: Keyword.fetch!(opts, :port),
      baud_rate: Keyword.get(opts, :baud_rate, 1_000_000),
      control_table: Keyword.get(opts, :control_table, Feetech.ControlTable.STS3215)
    )
  end

  @impl BB.Controller
  def handle_call(
        {:register_servo, servo_id, joint_name, center_angle, position_deadband, reverse?},
        _from,
        state
      ) do
    new_registry =
      Map.put(state.servo_registry, servo_id, %{
        joint_name: joint_name,
        center_angle: center_angle,
        position_deadband: position_deadband,
        reverse?: reverse?,
        last_position_raw: nil
      })

    BB.Safety.register(__MODULE__,
      robot: state.bb.robot,
      path: state.bb.path,
      opts: [
        feetech: state.feetech,
        servo_ids: Map.keys(new_registry),
        disarm_action: state.disarm_action
      ]
    )

    {:reply, :ok, %{state | servo_registry: new_registry}}
  end

  def handle_call({:read, servo_id, param}, _from, state) do
    result = Feetech.read(state.feetech, servo_id, param)
    {:reply, result, state}
  end

  def handle_call({:read_raw, servo_id, param}, _from, state) do
    result = Feetech.read_raw(state.feetech, servo_id, param)
    {:reply, result, state}
  end

  def handle_call({:write, servo_id, param, value}, _from, state) do
    result = Feetech.write(state.feetech, servo_id, param, value, await: true)
    {:reply, result, state}
  end

  def handle_call({:write_raw, servo_id, param, value}, _from, state) do
    result = Feetech.write_raw(state.feetech, servo_id, param, value, await: true)
    {:reply, result, state}
  end

  def handle_call({:ping, servo_id}, _from, state) do
    result = Feetech.ping(state.feetech, servo_id)
    {:reply, result, state}
  end

  def handle_call({:sync_read, servo_ids, param}, _from, state) do
    result = Feetech.sync_read(state.feetech, servo_ids, param)
    {:reply, result, state}
  end

  def handle_call(:list_servos, _from, state) do
    {:reply, {:ok, Map.keys(state.servo_registry)}, state}
  end

  def handle_call(:get_control_table, _from, state) do
    {:reply, {:ok, state.control_table}, state}
  end

  @impl BB.Controller
  def handle_cast({:write, servo_id, param, value}, state) do
    Feetech.write(state.feetech, servo_id, param, value)
    {:noreply, state}
  end

  def handle_cast({:write_raw, servo_id, param, value}, state) do
    Feetech.write_raw(state.feetech, servo_id, param, value)
    {:noreply, state}
  end

  def handle_cast({:sync_write, param, values}, state) do
    Feetech.sync_write(state.feetech, param, values)
    {:noreply, state}
  end

  @impl BB.Controller
  def handle_info(:start_polling, state) do
    schedule_poll(state.poll_interval_ms)
    schedule_status_poll(state.status_poll_interval_ms)
    {:noreply, state}
  end

  def handle_info(:poll, state) do
    state = poll_positions(state)
    schedule_poll(state.poll_interval_ms)
    {:noreply, state}
  end

  def handle_info(:poll_status, state) do
    state = poll_status(state)
    schedule_status_poll(state.status_poll_interval_ms)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: :armed}}}, state) do
    enable_all_torque(state)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{}}}, state) do
    {:noreply, state}
  end

  defp schedule_poll(interval_ms) do
    Process.send_after(self(), :poll, interval_ms)
  end

  defp schedule_status_poll(0), do: :ok

  defp schedule_status_poll(interval_ms) do
    Process.send_after(self(), :poll_status, interval_ms)
  end

  defp enable_all_torque(%{servo_registry: registry}) when map_size(registry) == 0 do
    :ok
  end

  defp enable_all_torque(state) do
    servo_ids = Map.keys(state.servo_registry) |> Enum.sort()

    # Disable torque so we can read positions without interference
    torque_off = Enum.map(servo_ids, fn id -> {id, 0} end)
    Feetech.sync_write_raw(state.feetech, :torque_enable, torque_off)

    # Read current positions in radians
    case Feetech.sync_read(state.feetech, servo_ids, :present_position) do
      {:ok, positions} ->
        # Buffer goal = present for each servo using reg_write.
        # reg_write stores data in a buffer WITHOUT writing to the register,
        # so it won't auto-enable torque like a direct write would.
        buffer_goal_positions(state.feetech, servo_ids, positions)

        # Trigger all buffered writes simultaneously.
        # When the goals are written atomically, torque auto-enables with the
        # correct goal already in place — no stale-goal impulse.
        Feetech.action(state.feetech)

      {:error, reason} ->
        Logger.warning("Failed to read positions before arming: #{inspect(reason)}")
    end

    # Explicitly enable torque and lock (in case auto-enable didn't fire)
    torque_on = Enum.map(servo_ids, fn id -> {id, 1} end)
    Feetech.sync_write_raw(state.feetech, :torque_enable, torque_on)
    lock_values = Enum.map(servo_ids, fn id -> {id, 1} end)
    Feetech.sync_write_raw(state.feetech, :lock, lock_values)
  end

  defp buffer_goal_positions(feetech, servo_ids, positions) do
    Enum.zip(servo_ids, positions)
    |> Enum.each(fn {id, position_rad} ->
      case Feetech.reg_write(feetech, id, :goal_position, position_rad) do
        :ok -> :ok
        {:ok, _} -> :ok
        error -> Logger.warning("Servo #{id} reg_write failed: #{inspect(error)}")
      end
    end)
  end

  defp poll_positions(%{servo_registry: registry} = state) when map_size(registry) == 0 do
    state
  end

  defp poll_positions(state) do
    servo_ids = Map.keys(state.servo_registry)

    case Feetech.sync_read(state.feetech, servo_ids, :present_position) do
      {:ok, positions} ->
        new_registry = publish_changed_positions(servo_ids, positions, state)
        %{state | servo_registry: new_registry}

      {:error, reason} ->
        Logger.warning("Failed to read positions: #{inspect(reason)}")
        emit_sync_read_error_diagnostic(state, reason)
        state
    end
  end

  defp publish_changed_positions(servo_ids, positions, state) do
    servo_ids
    |> Enum.zip(positions)
    |> Enum.reduce(state.servo_registry, fn {servo_id, position_rad}, registry ->
      maybe_publish_position(state, registry, servo_id, position_rad)
    end)
  end

  defp emit_sync_read_error_diagnostic(state, reason) do
    component = [state.bb.robot | state.bb.path]

    Diagnostic.error(component, "Communication error reading positions",
      values: %{reason: inspect(reason)}
    )
  end

  defp get_joint_name(state, servo_id) do
    case Map.get(state.servo_registry, servo_id) do
      %{joint_name: name} -> name
      nil -> String.to_atom("servo_#{servo_id}")
    end
  end

  defp maybe_publish_position(_state, registry, servo_id, _position_rad)
       when not is_map_key(registry, servo_id) do
    registry
  end

  defp maybe_publish_position(state, registry, servo_id, position_rad) do
    entry = Map.fetch!(registry, servo_id)
    maybe_publish_position_for_entry(state, registry, servo_id, position_rad, entry)
  end

  defp maybe_publish_position_for_entry(
         state,
         registry,
         servo_id,
         position_rad,
         %{last_position_raw: nil} = entry
       ) do
    publish_and_update(state, registry, servo_id, position_rad, entry)
  end

  defp maybe_publish_position_for_entry(
         state,
         registry,
         servo_id,
         position_rad,
         %{last_position_raw: last_rad, position_deadband: deadband} = entry
       ) do
    if position_exceeds_deadband?(position_rad, last_rad, deadband) do
      publish_and_update(state, registry, servo_id, position_rad, entry)
    else
      registry
    end
  end

  defp position_exceeds_deadband?(position_rad, last_rad, deadband) do
    deadband_rad = deadband * 2 * :math.pi() / @position_resolution
    abs(position_rad - last_rad) >= deadband_rad
  end

  defp publish_and_update(state, registry, servo_id, position_rad, entry) do
    joint_angle = radians_to_joint_angle(position_rad, entry.center_angle, entry.reverse?)
    publish_joint_state(state, entry.joint_name, joint_angle)
    Map.put(registry, servo_id, %{entry | last_position_raw: position_rad})
  end

  defp radians_to_joint_angle(position_rad, center_angle, reverse?) do
    servo_center_rad = :math.pi()
    servo_offset_rad = position_rad - servo_center_rad

    joint_offset_rad =
      if reverse? do
        -servo_offset_rad
      else
        servo_offset_rad
      end

    center_angle + joint_offset_rad
  end

  defp publish_joint_state(state, joint_name, position_rad) do
    {:ok, msg} =
      Message.new(JointState, joint_name,
        names: [joint_name],
        positions: [position_rad]
      )

    BB.publish(state.bb.robot, [:sensor, state.name, joint_name], msg)
  end

  defp poll_status(%{servo_registry: registry} = state) when map_size(registry) == 0 do
    state
  end

  defp poll_status(state) do
    servo_ids = Map.keys(state.servo_registry)

    temp_results = read_status_param(state, servo_ids, :present_temperature)
    voltage_results = read_status_param(state, servo_ids, :present_voltage)
    load_results = read_status_param(state, servo_ids, :present_load)
    error_results = read_status_param(state, servo_ids, :hardware_error_status)

    new_last_status =
      servo_ids
      |> Enum.with_index()
      |> Enum.reduce(state.last_status, fn {servo_id, index}, acc ->
        status = build_status(index, temp_results, voltage_results, load_results, error_results)

        last = Map.get(acc, servo_id)

        if status_changed?(status, last) do
          publish_servo_status(state, servo_id, status)
          maybe_report_hardware_error(state, servo_id, status, last)
          emit_status_diagnostics(state, servo_id, status, last)
          Map.put(acc, servo_id, status)
        else
          acc
        end
      end)

    %{state | last_status: new_last_status}
  end

  defp read_status_param(state, servo_ids, param) do
    case Feetech.sync_read(state.feetech, servo_ids, param) do
      {:ok, values} -> values
      {:error, _} -> List.duplicate(nil, length(servo_ids))
    end
  end

  defp build_status(index, temp_results, voltage_results, load_results, error_results) do
    %{
      temperature: Enum.at(temp_results, index),
      voltage: Enum.at(voltage_results, index),
      load: Enum.at(load_results, index),
      hardware_error: filter_hardware_error(Enum.at(error_results, index))
    }
  end

  defp filter_hardware_error(nil), do: nil

  defp filter_hardware_error(bits) when is_integer(bits),
    do: Bitwise.band(bits, Bitwise.bnot(0x10))

  @temp_deadband 1.0
  @voltage_deadband 0.1
  @load_deadband 5.0

  defp status_changed?(_status, nil), do: true

  defp status_changed?(status, last) do
    temp_changed?(status.temperature, last.temperature) or
      voltage_changed?(status.voltage, last.voltage) or
      load_changed?(status.load, last.load) or
      status.hardware_error != last.hardware_error
  end

  defp temp_changed?(nil, _), do: false
  defp temp_changed?(_, nil), do: true
  defp temp_changed?(new, old), do: abs(new - old) >= @temp_deadband

  defp voltage_changed?(nil, _), do: false
  defp voltage_changed?(_, nil), do: true
  defp voltage_changed?(new, old), do: abs(new - old) >= @voltage_deadband

  defp load_changed?(nil, _), do: false
  defp load_changed?(_, nil), do: true
  defp load_changed?(new, old), do: abs(new - old) >= @load_deadband

  defp publish_servo_status(state, servo_id, status) do
    case ServoStatus.new(state.name,
           servo_id: servo_id,
           temperature: status.temperature || 0.0,
           voltage: status.voltage || 0.0,
           load: status.load || 0.0,
           hardware_error: status.hardware_error
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:sensor, state.name, :servo_status], msg)

      {:error, reason} ->
        Logger.warning("Failed to create ServoStatus message: #{inspect(reason)}")
    end
  end

  defp emit_status_diagnostics(state, servo_id, status, last) do
    emit_temperature_diagnostic(state, servo_id, status.temperature, last)
    emit_voltage_diagnostic(state, servo_id, status.voltage, last)
  end

  defp emit_temperature_diagnostic(_state, _servo_id, nil, _last), do: :ok

  defp emit_temperature_diagnostic(state, servo_id, temp, last) do
    last_temp = if last, do: last.temperature, else: nil
    temp_level = temp_diagnostic_level(temp, last_temp)
    emit_temp_diagnostic_for_level(temp_level, state, servo_id, temp)
  end

  defp temp_diagnostic_level(temp, last_temp) do
    crossed_error? = crossed_threshold?(temp, last_temp, @temp_error_threshold, :up)
    crossed_warning? = crossed_threshold?(temp, last_temp, @temp_warning_threshold, :up)
    recovered? = crossed_threshold?(temp, last_temp, @temp_warning_threshold, :down)

    cond do
      crossed_error? -> :error
      crossed_warning? -> :warn
      recovered? -> :ok
      true -> nil
    end
  end

  defp emit_temp_diagnostic_for_level(nil, _state, _servo_id, _temp), do: :ok

  defp emit_temp_diagnostic_for_level(:error, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.error(component, "Temperature critical",
      values: %{temperature: temp, threshold: @temp_error_threshold, servo_id: servo_id}
    )
  end

  defp emit_temp_diagnostic_for_level(:warn, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Temperature elevated",
      values: %{temperature: temp, threshold: @temp_warning_threshold, servo_id: servo_id}
    )
  end

  defp emit_temp_diagnostic_for_level(:ok, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.ok(component, "Temperature normal",
      values: %{temperature: temp, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic(_state, _servo_id, nil, _last), do: :ok

  defp emit_voltage_diagnostic(state, servo_id, voltage, last) do
    last_voltage = if last, do: last.voltage, else: nil
    voltage_level = voltage_diagnostic_level(voltage, last_voltage)
    emit_voltage_diagnostic_for_level(voltage_level, state, servo_id, voltage)
  end

  defp voltage_diagnostic_level(voltage, last_voltage) do
    crossed_low? = crossed_threshold?(voltage, last_voltage, @voltage_low_threshold, :down)
    crossed_high? = crossed_threshold?(voltage, last_voltage, @voltage_high_threshold, :up)
    recovered? = voltage_recovered?(voltage, last_voltage)

    cond do
      crossed_low? -> :warn_low
      crossed_high? -> :warn_high
      recovered? -> :ok
      true -> nil
    end
  end

  defp voltage_recovered?(_voltage, nil), do: false

  defp voltage_recovered?(voltage, last_voltage) do
    voltage_ok? = voltage >= @voltage_low_threshold and voltage <= @voltage_high_threshold

    last_voltage_ok? =
      last_voltage >= @voltage_low_threshold and last_voltage <= @voltage_high_threshold

    voltage_ok? and not last_voltage_ok?
  end

  defp emit_voltage_diagnostic_for_level(nil, _state, _servo_id, _voltage), do: :ok

  defp emit_voltage_diagnostic_for_level(:warn_low, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Voltage low",
      values: %{voltage: voltage, threshold: @voltage_low_threshold, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic_for_level(:warn_high, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Voltage high",
      values: %{voltage: voltage, threshold: @voltage_high_threshold, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic_for_level(:ok, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.ok(component, "Voltage normal", values: %{voltage: voltage, servo_id: servo_id})
  end

  defp crossed_threshold?(value, nil, threshold, :up), do: value >= threshold
  defp crossed_threshold?(value, nil, threshold, :down), do: value < threshold

  defp crossed_threshold?(value, last, threshold, :up),
    do: value >= threshold and last < threshold

  defp crossed_threshold?(value, last, threshold, :down),
    do: value < threshold and last >= threshold

  defp maybe_report_hardware_error(_state, _servo_id, %{hardware_error: nil}, _last), do: :ok
  defp maybe_report_hardware_error(_state, _servo_id, %{hardware_error: 0}, _last), do: :ok

  defp maybe_report_hardware_error(state, servo_id, %{hardware_error: error}, nil) do
    report_hardware_error(state, servo_id, error)
  end

  defp maybe_report_hardware_error(state, servo_id, %{hardware_error: error}, %{
         hardware_error: last_error
       })
       when error != last_error do
    report_hardware_error(state, servo_id, error)
  end

  defp maybe_report_hardware_error(_state, _servo_id, _status, _last), do: :ok

  defp report_hardware_error(state, servo_id, error) do
    joint_name = get_joint_name(state, servo_id)
    path = state.bb.path ++ [joint_name]
    alert = HardwareAlert.from_bits(servo_id, error)

    BB.Safety.report_error(state.bb.robot, path, alert)

    component = [state.bb.robot | path]

    Diagnostic.error(component, "Hardware error detected",
      values: %{servo_id: servo_id, error_bits: error, alerts: alert.alerts}
    )
  end

  @impl BB.Controller
  def terminate(_reason, state) do
    if Process.alive?(state.feetech) do
      Feetech.stop(state.feetech)
    end

    :ok
  end
end
