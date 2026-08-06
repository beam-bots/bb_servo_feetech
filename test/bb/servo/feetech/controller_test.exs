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
    test "disables torque and lock on all registered servo IDs with acknowledged writes" do
      feetech_pid = self()

      Feetech
      |> expect(:write_raw, 4, fn ^feetech_pid, id, register, 0, opts ->
        assert id in [1, 2]
        assert register in [:torque_enable, :lock]
        assert Keyword.fetch!(opts, :await_response)
        {:ok, %{}}
      end)

      opts = [
        feetech: feetech_pid,
        servo_ids: [1, 2],
        disarm_action: :disable_torque
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok when no servos registered" do
      Feetech |> reject(:write_raw, 5)

      opts = [
        feetech: self(),
        servo_ids: [],
        disarm_action: :disable_torque
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok with :hold action without disabling torque" do
      Feetech |> reject(:write_raw, 5)

      opts = [
        feetech: self(),
        servo_ids: [1, 2],
        disarm_action: :hold
      ]

      assert :ok = Controller.disarm(opts)
    end

    test "returns an error when a torque-disable write is rejected" do
      feetech_pid = self()

      Feetech
      |> stub(:write_raw, fn ^feetech_pid, id, :torque_enable, 0, _opts ->
        if id == 2, do: {:error, :timeout}, else: {:ok, %{}}
      end)

      opts = [
        feetech: feetech_pid,
        servo_ids: [1, 2],
        disarm_action: :disable_torque
      ]

      assert {:error, {:servo, 2, :torque_enable, :timeout}} = Controller.disarm(opts)
    end

    test "returns an error when the feetech process is dead" do
      {:ok, dead_pid} = Agent.start(fn -> :ok end)
      Agent.stop(dead_pid)

      opts = [
        feetech: dead_pid,
        servo_ids: [1, 2],
        disarm_action: :disable_torque
      ]

      assert {:error, {:exit, _reason}} = Controller.disarm(opts)
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
      assert state.loop.clock == {:rate, 100.0}
      assert state.loop.period_ns == 10_000_000

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

      message = {:register_servo, 1, [:shoulder, :servo], 2}

      assert {:reply, {:ok, servo_table}, new_state} =
               Controller.handle_call(message, {self(), make_ref()}, state)

      assert servo_table == state.servo_table
      assert new_state.servo_ids == [1]

      [
        {1, actuator_path, deadband, last_pos, present_pos, present_temp, present_voltage,
         present_load, hw_error, pending_writes, pending_limit, torque_enabled}
      ] =
        :ets.lookup(state.servo_table, 1)

      assert actuator_path == [:shoulder, :servo]
      assert deadband == 2
      assert last_pos == nil
      assert present_pos == nil
      assert present_temp == nil
      assert present_voltage == nil
      assert present_load == nil
      assert hw_error == nil
      assert pending_writes == nil
      assert pending_limit == nil
      assert torque_enabled == false
    end

    test "refuses a servo ID another actuator already drives", %{state: state} do
      first = {:register_servo, 1, [:shoulder, :servo], 2}
      {:reply, {:ok, _}, state} = Controller.handle_call(first, {self(), make_ref()}, state)

      second = {:register_servo, 1, [:elbow, :servo], 2}

      assert {:reply, {:error, error}, new_state} =
               Controller.handle_call(second, {self(), make_ref()}, state)

      assert %BB.Error.Invalid.Feetech.DuplicateServoId{
               servo_id: 1,
               actuator_path: [:elbow, :servo],
               registered_path: [:shoulder, :servo]
             } = error

      # The first actuator keeps its row and its feedback.
      assert [{1, [:shoulder, :servo], _, _, _, _, _, _, _, _, _, _}] =
               :ets.lookup(state.servo_table, 1)

      assert new_state.servo_ids == [1]
    end

    test "lets an actuator that restarted re-register its own servo", %{state: state} do
      message = {:register_servo, 1, [:shoulder, :servo], 2}
      {:reply, {:ok, _}, state} = Controller.handle_call(message, {self(), make_ref()}, state)

      # Whatever the previous incarnation left behind is stale.
      :ets.update_element(state.servo_table, 1, [{10, [goal_position: 3000]}, {12, true}])

      assert {:reply, {:ok, _table}, _state} =
               Controller.handle_call(message, {self(), make_ref()}, state)

      assert [{1, [:shoulder, :servo], _, _, _, _, _, _, _, nil, nil, false}] =
               :ets.lookup(state.servo_table, 1)
    end

    test "registers multiple servos", %{state: state} do
      message1 = {:register_servo, 1, [:shoulder, :servo], 2}
      {:reply, {:ok, _}, state} = Controller.handle_call(message1, {self(), make_ref()}, state)

      message2 = {:register_servo, 2, [:elbow, :servo], 3}

      {:reply, {:ok, _}, new_state} =
        Controller.handle_call(message2, {self(), make_ref()}, state)

      assert new_state.servo_ids == [1, 2]

      [{1, [:shoulder, :servo], _, _, _, _, _, _, _, _, _, _}] =
        :ets.lookup(state.servo_table, 1)

      [{2, [:elbow, :servo], _, _, _, _, _, _, _, _, _, _}] = :ets.lookup(state.servo_table, 2)
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
      |> expect(:write, fn pid, 1, :torque_enable, true, [await_response: true]
                           when is_pid(pid) ->
        :ok
      end)

      message = {:write, 1, :torque_enable, true}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "forwards write_raw to Feetech with await", %{state: state} do
      Feetech
      |> expect(:write_raw, fn pid, 1, :goal_position, 2048, [await_response: true]
                               when is_pid(pid) ->
        :ok
      end)

      message = {:write_raw, 1, :goal_position, 2048}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "torque state caching" do
    setup :controller_state_with_servos

    test "a torque_enable write updates the cache", %{state: state, servo_table: servo_table} do
      Feetech
      |> expect(:write, fn _pid, 1, :torque_enable, true, _opts -> :ok end)

      Controller.handle_call({:write, 1, :torque_enable, true}, {self(), make_ref()}, state)

      assert torque_enabled?(servo_table, 1)
    end

    test "a raw torque_enable write updates the cache", %{
      state: state,
      servo_table: servo_table
    } do
      :ets.update_element(servo_table, 1, [{12, true}])

      Feetech
      |> expect(:write_raw, fn _pid, 1, :torque_enable, 0, _opts -> :ok end)

      Controller.handle_call({:write_raw, 1, :torque_enable, 0}, {self(), make_ref()}, state)

      refute torque_enabled?(servo_table, 1)
    end

    test "a failed write leaves the cache alone", %{state: state, servo_table: servo_table} do
      Feetech
      |> expect(:write, fn _pid, 1, :torque_enable, true, _opts -> {:error, :timeout} end)

      Controller.handle_call({:write, 1, :torque_enable, true}, {self(), make_ref()}, state)

      refute torque_enabled?(servo_table, 1)
    end

    test "writes to other params don't touch it", %{state: state, servo_table: servo_table} do
      Feetech
      |> expect(:write_raw, fn _pid, 1, :goal_position, 2048, _opts -> :ok end)

      Controller.handle_call({:write_raw, 1, :goal_position, 2048}, {self(), make_ref()}, state)

      refute torque_enabled?(servo_table, 1)
    end
  end

  describe "handle_call/3 - resume_servo" do
    setup :controller_state_with_servos

    test "writes the goal before turning torque on", %{state: state} do
      test_pid = self()

      Feetech
      |> expect(:write_raw, fn _pid, 1, :goal_position, 2048, [await_response: true] ->
        send(test_pid, :goal_position)
        :ok
      end)
      |> expect(:write, fn _pid, 1, :torque_enable, true, [await_response: true] ->
        send(test_pid, :torque_enable)
        :ok
      end)

      message = {:resume_servo, 1, [{:goal_position, 2048}]}
      assert {:reply, :ok, _state} = Controller.handle_call(message, {self(), make_ref()}, state)

      assert drain_messages() == [:goal_position, :torque_enable]
    end

    test "resolves :present_position against the bus", %{state: state} do
      Feetech
      |> expect(:read_raw, fn _pid, 1, :present_position -> {:ok, 1234} end)
      |> expect(:write_raw, fn _pid, 1, :goal_position, 1234, _opts -> :ok end)
      |> expect(:write, fn _pid, 1, :torque_enable, true, _opts -> :ok end)

      message = {:resume_servo, 1, [{:present_position, nil}]}
      assert {:reply, :ok, _state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "caches the torque state and clears the pending write", %{
      state: state,
      servo_table: servo_table
    } do
      :ets.update_element(servo_table, 1, [{10, [goal_position: 9999]}])

      Feetech
      |> expect(:write_raw, fn _pid, 1, :goal_position, 2048, _opts -> :ok end)
      |> expect(:write, fn _pid, 1, :torque_enable, true, _opts -> :ok end)

      message = {:resume_servo, 1, [{:goal_position, 2048}]}
      assert {:reply, :ok, _state} = Controller.handle_call(message, {self(), make_ref()}, state)

      [{1, _, _, _, _, _, _, _, _, pending_writes, _, _}] = :ets.lookup(servo_table, 1)
      assert pending_writes == nil
      assert torque_enabled?(servo_table, 1)
    end

    test "torque stays off when the goal write fails", %{
      state: state,
      servo_table: servo_table
    } do
      Feetech
      |> expect(:write_raw, fn _pid, 1, :goal_position, 2048, _opts -> {:error, :timeout} end)
      |> reject(:write, 5)

      message = {:resume_servo, 1, [{:goal_position, 2048}]}

      assert {:reply, {:error, {:servo, 1, :goal_position, :timeout}}, _state} =
               Controller.handle_call(message, {self(), make_ref()}, state)

      refute torque_enabled?(servo_table, 1)
    end

    test "a converted param goes through the control table", %{state: state} do
      Feetech
      |> expect(:write, fn _pid, 1, :goal_speed, -1.5, [await_response: true] -> :ok end)
      |> expect(:write, fn _pid, 1, :torque_enable, true, _opts -> :ok end)

      message = {:resume_servo, 1, [{:goal_speed, -1.5}]}
      assert {:reply, :ok, _state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_call/3 - ping and list operations" do
    setup do
      base = controller_state(%{})

      :ets.insert(
        base.servo_table,
        {1, [:shoulder, :servo], 2, nil, nil, nil, nil, nil, nil, nil, nil, false}
      )

      :ets.insert(
        base.servo_table,
        {2, [:elbow, :servo], 2, nil, nil, nil, nil, nil, nil, nil, nil, false}
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

    test "batches each pending param into one sync_write across servos", %{state: state} do
      :ets.update_element(state.servo_table, 1, [
        {10, [goal_speed: 0.0, goal_position: 2048]}
      ])

      :ets.update_element(state.servo_table, 2, [
        {10, [goal_speed: 1.5, goal_position: 3000]}
      ])

      Feetech
      |> expect(:sync_write, fn pid, :goal_speed, values when is_pid(pid) ->
        assert {1, 0.0} in values
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

      [{1, _, _, _, _, _, _, _, _, pending_writes, _, _}] =
        :ets.lookup(state.servo_table, 1)

      assert pending_writes == nil
    end

    test "writes the torque ceiling before the goal that might reach it", %{state: state} do
      :ets.update_element(state.servo_table, 1, [
        {10, [goal_position: 2048]},
        {11, 0.25}
      ])

      test_pid = self()

      Feetech
      |> expect(:sync_write, fn _pid, :torque_limit, [{1, 0.25}] ->
        send(test_pid, :torque_limit)
        :ok
      end)
      |> expect(:sync_write_raw, fn _pid, :goal_position, [{1, 2048}] ->
        send(test_pid, :goal_position)
        :ok
      end)
      |> expect(:sync_read, fn _pid, [1, 2], :present_position -> {:ok, [3.14, 1.57]} end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      assert drain_messages() == [:torque_limit, :goal_position]
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

      BB.Actuator
      |> stub(:to_joint_space, fn _robot, _path, msg -> msg end)

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

      BB.Actuator
      |> stub(:to_joint_space, fn _robot, _path, msg -> msg end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      [{1, _, _, last_pos, present_pos, _, _, _, _, _, _, _}] =
        :ets.lookup(state.servo_table, 1)

      assert last_pos == 3.14
      assert present_pos == 3.14
    end

    test "applies position deadband filtering", %{state: state} do
      :ets.update_element(state.servo_table, 1, [{4, 3.14}])
      :ets.update_element(state.servo_table, 2, [{4, 1.57}])

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

    bb = %{robot: TestRobot, path: [:feetech]}

    state = %{
      bb: bb,
      feetech: feetech_pid,
      control_table: @control_table,
      name: :feetech,
      loop: BB.Loop.new(bb, clock: {:rate, 100}),
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
      {1, [:shoulder, :servo], 2, nil, nil, nil, nil, nil, nil, nil, nil, false}
    )

    :ets.insert(
      base.servo_table,
      {2, [:elbow, :servo], 2, nil, nil, nil, nil, nil, nil, nil, nil, false}
    )

    BB.Actuator
    |> stub(:to_joint_space, fn _robot, _path, msg -> msg end)

    state = %{base.state | servo_ids: [1, 2]}

    %{state: state, servo_table: base.servo_table}
  end

  defp torque_enabled?(servo_table, servo_id) do
    [{^servo_id, _, _, _, _, _, _, _, _, _, _, torque_enabled}] =
      :ets.lookup(servo_table, servo_id)

    torque_enabled
  end

  defp drain_messages(acc \\ []) do
    receive do
      message -> drain_messages([message | acc])
    after
      0 -> Enum.reverse(acc)
    end
  end
end
