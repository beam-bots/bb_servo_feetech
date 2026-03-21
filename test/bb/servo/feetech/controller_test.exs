# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.ControllerTest do
  use ExUnit.Case, async: false
  use Mimic

  alias BB.Servo.Feetech.Controller

  @control_table Feetech.ControlTable.STS3215

  setup :verify_on_exit!

  describe "disarm/1" do
    test "disables torque on all registered servo IDs" do
      feetech_pid = self()

      Feetech
      |> expect(:sync_write, fn ^feetech_pid, :torque_enable, values ->
        assert {1, false} in values
        assert {2, false} in values
        :ok
      end)
      |> expect(:sync_write, fn ^feetech_pid, :lock, values ->
        assert {1, false} in values
        assert {2, false} in values
        :ok
      end)

      opts = [
        feetech: feetech_pid,
        servo_ids: [1, 2],
        disarm_action: :disable_torque
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok when no servos registered" do
      opts = [
        feetech: self(),
        servo_ids: [],
        disarm_action: :disable_torque
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok with :hold action without disabling torque" do
      opts = [
        feetech: self(),
        servo_ids: [1, 2],
        disarm_action: :hold
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok when feetech process is dead" do
      {:ok, dead_pid} = Agent.start(fn -> :ok end)
      Agent.stop(dead_pid)

      opts = [
        feetech: dead_pid,
        servo_ids: [1, 2],
        disarm_action: :disable_torque
      ]

      assert :ok = Controller.disarm(opts)
    end
  end

  describe "init/1" do
    setup do
      feetech_pid = spawn(fn -> Process.sleep(:infinity) end)

      Feetech
      |> stub(:start_link, fn _opts -> {:ok, feetech_pid} end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      BB.Safety
      |> stub(:register, fn _module, _opts -> :ok end)

      on_exit(fn ->
        if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
      end)

      %{feetech_pid: feetech_pid}
    end

    test "starts feetech driver with correct options", %{feetech_pid: feetech_pid} do
      Feetech
      |> expect(:start_link, fn opts ->
        assert Keyword.get(opts, :port) == "/dev/ttyUSB0"
        assert Keyword.get(opts, :baud_rate) == 1_000_000
        assert Keyword.get(opts, :control_table) == @control_table
        {:ok, feetech_pid}
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000,
        control_table: @control_table
      ]

      assert {:ok, state} = Controller.init(opts)
      assert state.feetech == feetech_pid
      assert state.control_table == @control_table
      assert state.name == :feetech
    end

    test "creates ETS table", %{feetech_pid: _feetech_pid} do
      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0"
      ]

      assert {:ok, state} = Controller.init(opts)
      assert is_reference(state.servo_table)
      assert :ets.info(state.servo_table) != :undefined
      assert state.servo_ids == []

      :ets.delete(state.servo_table)
    end

    test "uses default loop interval of 10ms" do
      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0"
      ]

      assert {:ok, state} = Controller.init(opts)
      assert state.loop_interval_ms == 10

      :ets.delete(state.servo_table)
    end

    test "registers with safety system", %{feetech_pid: feetech_pid} do
      BB.Safety
      |> expect(:register, fn Controller, opts ->
        assert Keyword.get(opts, :robot) == TestRobot
        assert Keyword.get(opts, :path) == [:feetech]
        assert Keyword.get(opts, :opts)[:feetech] == feetech_pid
        :ok
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0"
      ]

      assert {:ok, state} = Controller.init(opts)

      :ets.delete(state.servo_table)
    end

    test "subscribes to state machine transitions" do
      BB
      |> expect(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0"
      ]

      assert {:ok, state} = Controller.init(opts)

      :ets.delete(state.servo_table)
    end

    test "returns error when feetech fails to start" do
      Feetech
      |> expect(:start_link, fn _opts -> {:error, :port_not_found} end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/nonexistent"
      ]

      assert {:stop, :port_not_found} = Controller.init(opts)
    end
  end

  describe "handle_call/3 - servo registration" do
    setup :controller_state

    test "registers a servo in ETS and updates servo_ids", %{state: state} do
      BB.Safety
      |> expect(:register, fn Controller, opts ->
        assert 1 in Keyword.get(opts, :opts)[:servo_ids]
        :ok
      end)

      message = {:register_servo, 1, :shoulder, 0.0, 2, false}

      assert {:reply, {:ok, servo_table}, new_state} =
               Controller.handle_call(message, {self(), make_ref()}, state)

      assert servo_table == state.servo_table
      assert new_state.servo_ids == [1]

      [
        {1, joint_name, center_angle, reverse?, deadband, last_pos, present_pos, present_temp,
         present_voltage, present_load, hw_error, goal_pos, goal_speed}
      ] =
        :ets.lookup(state.servo_table, 1)

      assert joint_name == :shoulder
      assert center_angle == 0.0
      assert reverse? == false
      assert deadband == 2
      assert last_pos == nil
      assert present_pos == nil
      assert present_temp == nil
      assert present_voltage == nil
      assert present_load == nil
      assert hw_error == nil
      assert goal_pos == nil
      assert goal_speed == nil
    end

    test "registers multiple servos", %{state: state} do
      message1 = {:register_servo, 1, :shoulder, 0.0, 2, false}
      {:reply, {:ok, _}, state} = Controller.handle_call(message1, {self(), make_ref()}, state)

      message2 = {:register_servo, 2, :elbow, 1.57, 3, true}

      {:reply, {:ok, _}, new_state} =
        Controller.handle_call(message2, {self(), make_ref()}, state)

      assert new_state.servo_ids == [1, 2]

      [{1, :shoulder, _, _, _, _, _, _, _, _, _, _, _}] = :ets.lookup(state.servo_table, 1)
      [{2, :elbow, _, true, _, _, _, _, _, _, _, _, _}] = :ets.lookup(state.servo_table, 2)
    end
  end

  describe "handle_call/3 - read operations" do
    setup :controller_state

    test "forwards read to Feetech", %{state: state} do
      Feetech
      |> expect(:read, fn pid, 1, :present_position when is_pid(pid) -> {:ok, 1.57} end)

      message = {:read, 1, :present_position}

      assert {:reply, {:ok, 1.57}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "forwards read_raw to Feetech", %{state: state} do
      Feetech
      |> expect(:read_raw, fn pid, 1, :present_position when is_pid(pid) -> {:ok, 2048} end)

      message = {:read_raw, 1, :present_position}

      assert {:reply, {:ok, 2048}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_call/3 - write operations" do
    setup :controller_state

    test "forwards write to Feetech with await", %{state: state} do
      Feetech
      |> expect(:write, fn pid, 1, :torque_enable, true, [await: true] when is_pid(pid) ->
        :ok
      end)

      message = {:write, 1, :torque_enable, true}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "forwards write_raw to Feetech with await", %{state: state} do
      Feetech
      |> expect(:write_raw, fn pid, 1, :goal_position, 2048, [await: true] when is_pid(pid) ->
        :ok
      end)

      message = {:write_raw, 1, :goal_position, 2048}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_call/3 - ping and list operations" do
    setup do
      base = controller_state(%{})

      :ets.insert(
        base.servo_table,
        {1, :shoulder, 0.0, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
      )

      :ets.insert(
        base.servo_table,
        {2, :elbow, 1.57, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
      )

      state = %{base.state | servo_ids: [1, 2]}

      %{state: state, servo_table: base.servo_table}
    end

    test "forwards ping to Feetech", %{state: state} do
      Feetech
      |> expect(:ping, fn pid, 1 when is_pid(pid) ->
        {:ok, %{errors: [], torque_enabled: false}}
      end)

      message = {:ping, 1}

      assert {:reply, {:ok, _}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "returns registered servo IDs", %{state: state} do
      message = :list_servos

      {:reply, {:ok, servo_ids}, ^state} =
        Controller.handle_call(message, {self(), make_ref()}, state)

      assert servo_ids == [1, 2]
    end

    test "returns control table", %{state: state} do
      message = :get_control_table

      assert {:reply, {:ok, @control_table}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_info(:tick) - command processing" do
    setup :controller_state_with_servos

    test "batches pending goal positions into sync_write_raw", %{state: state} do
      :ets.update_element(state.servo_table, 1, [{12, 2048}, {13, 0}])
      :ets.update_element(state.servo_table, 2, [{12, 3000}, {13, 1.5}])

      Feetech
      |> expect(:sync_write, fn pid, :goal_speed, values when is_pid(pid) ->
        assert {1, 0} in values
        assert {2, 1.5} in values
        :ok
      end)
      |> expect(:sync_write_raw, fn pid, :goal_position, values when is_pid(pid) ->
        assert {1, 2048} in values
        assert {2, 3000} in values
        :ok
      end)
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      [{1, _, _, _, _, _, _, _, _, _, _, goal_pos, goal_speed}] =
        :ets.lookup(state.servo_table, 1)

      assert goal_pos == nil
      assert goal_speed == nil
    end

    test "skips sync_write when no commands pending", %{state: state} do
      Feetech
      |> reject(:sync_write, 3)
      |> reject(:sync_write_raw, 3)
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)
    end

    test "reads positions and publishes JointState", %{state: state} do
      Feetech
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)

      BB
      |> expect(:publish, 2, fn TestRobot, [:sensor, :feetech, joint_name], msg ->
        assert joint_name in [:shoulder, :elbow]
        assert msg.payload.__struct__ == BB.Message.Sensor.JointState
        :ok
      end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)
    end

    test "updates last_position_raw in ETS after publishing", %{state: state} do
      Feetech
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      [{1, _, _, _, _, last_pos, present_pos, _, _, _, _, _, _}] =
        :ets.lookup(state.servo_table, 1)

      assert last_pos == 3.14
      assert present_pos == 3.14
    end

    test "applies position deadband filtering", %{state: state} do
      :ets.update_element(state.servo_table, 1, [{6, 3.14}])
      :ets.update_element(state.servo_table, 2, [{6, 1.57}])

      Feetech
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.1401, 1.5701]}
      end)

      BB
      |> reject(:publish, 3)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)
    end
  end

  describe "handle_info(:tick) - status polling" do
    setup do
      base = controller_state_with_servos(%{})
      state = %{base.state | status_every_n_ticks: 3, status_tick_counter: 0}
      %{state: state, servo_table: base.servo_table}
    end

    test "increments status counter each tick", %{state: state} do
      Feetech
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, new_state} = Controller.handle_info(:tick, state)
      assert new_state.status_tick_counter == 1
    end

    test "polls status when counter reaches threshold", %{state: state} do
      state = %{state | status_tick_counter: 2}

      Feetech
      |> expect(:sync_read, fn pid, [1, 2], :present_position when is_pid(pid) ->
        {:ok, [3.14, 1.57]}
      end)
      |> expect(:sync_read, fn pid, [1, 2], :present_temperature when is_pid(pid) ->
        {:ok, [35.0, 36.0]}
      end)
      |> expect(:sync_read, fn pid, [1, 2], :present_voltage when is_pid(pid) ->
        {:ok, [7.0, 7.1]}
      end)
      |> expect(:sync_read, fn pid, [1, 2], :present_load when is_pid(pid) ->
        {:ok, [10.0, 15.0]}
      end)
      |> expect(:sync_read, fn pid, [1, 2], :hardware_error_status when is_pid(pid) ->
        {:ok, [0, 0]}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, new_state} = Controller.handle_info(:tick, state)
      assert new_state.status_tick_counter == 0
    end
  end

  defp controller_state(_context \\ %{}) do
    feetech_pid = spawn(fn -> Process.sleep(:infinity) end)
    servo_table = :ets.new(:test_servo_table, [:set, :public])

    state = %{
      bb: %{robot: TestRobot, path: [:feetech]},
      feetech: feetech_pid,
      control_table: @control_table,
      name: :feetech,
      loop_interval_ms: 10,
      status_poll_interval_ms: 1000,
      status_every_n_ticks: 100,
      status_tick_counter: 0,
      disarm_action: :disable_torque,
      servo_table: servo_table,
      servo_ids: [],
      last_status: %{}
    }

    BB.Safety
    |> stub(:register, fn _module, _opts -> :ok end)

    on_exit(fn ->
      if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
    end)

    %{state: state, servo_table: servo_table}
  end

  defp controller_state_with_servos(_context) do
    base = controller_state()

    :ets.insert(
      base.servo_table,
      {1, :shoulder, 0.0, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
    )

    :ets.insert(
      base.servo_table,
      {2, :elbow, 1.57, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
    )

    state = %{base.state | servo_ids: [1, 2]}

    %{state: state, servo_table: base.servo_table}
  end
end
