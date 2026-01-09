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

      assert {:ok, _state} = Controller.init(opts)
    end

    test "subscribes to state machine transitions" do
      BB
      |> expect(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech]},
        port: "/dev/ttyUSB0"
      ]

      assert {:ok, _state} = Controller.init(opts)
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
    setup do
      state = %{
        bb: %{robot: TestRobot, path: [:feetech]},
        feetech: self(),
        control_table: @control_table,
        name: :feetech,
        poll_interval_ms: 50,
        status_poll_interval_ms: 1000,
        disarm_action: :disable_torque,
        servo_registry: %{},
        last_status: %{}
      }

      BB.Safety
      |> stub(:register, fn _module, _opts -> :ok end)

      %{state: state}
    end

    test "registers a servo and updates registry", %{state: state} do
      BB.Safety
      |> expect(:register, fn Controller, opts ->
        assert 1 in Keyword.get(opts, :opts)[:servo_ids]
        :ok
      end)

      message = {:register_servo, 1, :shoulder, 0.0, 2, false}

      assert {:reply, :ok, new_state} =
               Controller.handle_call(message, {self(), make_ref()}, state)

      assert Map.has_key?(new_state.servo_registry, 1)
      servo = new_state.servo_registry[1]
      assert servo.joint_name == :shoulder
      assert servo.center_angle == 0.0
      assert servo.position_deadband == 2
      assert servo.reverse? == false
      assert servo.last_position_raw == nil
    end

    test "registers multiple servos", %{state: state} do
      message1 = {:register_servo, 1, :shoulder, 0.0, 2, false}
      {:reply, :ok, state} = Controller.handle_call(message1, {self(), make_ref()}, state)

      message2 = {:register_servo, 2, :elbow, 1.57, 3, true}
      {:reply, :ok, new_state} = Controller.handle_call(message2, {self(), make_ref()}, state)

      assert Map.has_key?(new_state.servo_registry, 1)
      assert Map.has_key?(new_state.servo_registry, 2)
      assert new_state.servo_registry[2].joint_name == :elbow
      assert new_state.servo_registry[2].reverse? == true
    end
  end

  describe "handle_call/3 - read operations" do
    setup do
      feetech_pid = spawn(fn -> Process.sleep(:infinity) end)

      state = %{
        bb: %{robot: TestRobot, path: [:feetech]},
        feetech: feetech_pid,
        control_table: @control_table,
        name: :feetech,
        servo_registry: %{},
        last_status: %{}
      }

      on_exit(fn ->
        if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
      end)

      %{state: state, feetech_pid: feetech_pid}
    end

    test "forwards read to Feetech", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:read, fn ^feetech_pid, 1, :present_position -> {:ok, 1.57} end)

      message = {:read, 1, :present_position}

      assert {:reply, {:ok, 1.57}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "forwards read_raw to Feetech", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:read_raw, fn ^feetech_pid, 1, :present_position -> {:ok, 2048} end)

      message = {:read_raw, 1, :present_position}

      assert {:reply, {:ok, 2048}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_call/3 - write operations" do
    setup do
      feetech_pid = spawn(fn -> Process.sleep(:infinity) end)

      state = %{
        bb: %{robot: TestRobot, path: [:feetech]},
        feetech: feetech_pid,
        control_table: @control_table,
        name: :feetech,
        servo_registry: %{},
        last_status: %{}
      }

      on_exit(fn ->
        if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
      end)

      %{state: state, feetech_pid: feetech_pid}
    end

    test "forwards write to Feetech with await", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:write, fn ^feetech_pid, 1, :torque_enable, true, [await: true] -> :ok end)

      message = {:write, 1, :torque_enable, true}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "forwards write_raw to Feetech with await", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:write_raw, fn ^feetech_pid, 1, :goal_position, 2048, [await: true] -> :ok end)

      message = {:write_raw, 1, :goal_position, 2048}
      assert {:reply, :ok, ^state} = Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_call/3 - ping and list operations" do
    setup do
      feetech_pid = spawn(fn -> Process.sleep(:infinity) end)

      state = %{
        bb: %{robot: TestRobot, path: [:feetech]},
        feetech: feetech_pid,
        control_table: @control_table,
        name: :feetech,
        servo_registry: %{1 => %{}, 2 => %{}},
        last_status: %{}
      }

      on_exit(fn ->
        if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
      end)

      %{state: state, feetech_pid: feetech_pid}
    end

    test "forwards ping to Feetech", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:ping, fn ^feetech_pid, 1 -> {:ok, %{errors: [], torque_enabled: false}} end)

      message = {:ping, 1}

      assert {:reply, {:ok, _}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end

    test "returns registered servo IDs", %{state: state} do
      message = :list_servos

      {:reply, {:ok, servo_ids}, ^state} =
        Controller.handle_call(message, {self(), make_ref()}, state)

      assert 1 in servo_ids
      assert 2 in servo_ids
    end

    test "returns control table", %{state: state} do
      message = :get_control_table

      assert {:reply, {:ok, @control_table}, ^state} =
               Controller.handle_call(message, {self(), make_ref()}, state)
    end
  end

  describe "handle_cast/2 - async write operations" do
    setup do
      feetech_pid = spawn(fn -> Process.sleep(:infinity) end)

      state = %{
        bb: %{robot: TestRobot, path: [:feetech]},
        feetech: feetech_pid,
        control_table: @control_table,
        name: :feetech,
        servo_registry: %{},
        last_status: %{}
      }

      on_exit(fn ->
        if Process.alive?(feetech_pid), do: Process.exit(feetech_pid, :kill)
      end)

      %{state: state, feetech_pid: feetech_pid}
    end

    test "forwards async write to Feetech", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:write, fn ^feetech_pid, 1, :goal_position, 1.57 -> :ok end)

      message = {:write, 1, :goal_position, 1.57}
      assert {:noreply, ^state} = Controller.handle_cast(message, state)
    end

    test "forwards async write_raw to Feetech", %{state: state, feetech_pid: feetech_pid} do
      Feetech
      |> expect(:write_raw, fn ^feetech_pid, 1, :goal_position, 2048 -> :ok end)

      message = {:write_raw, 1, :goal_position, 2048}
      assert {:noreply, ^state} = Controller.handle_cast(message, state)
    end

    test "forwards sync_write to Feetech", %{state: state, feetech_pid: feetech_pid} do
      values = [{1, 2048}, {2, 1024}]

      Feetech
      |> expect(:sync_write, fn ^feetech_pid, :goal_position, ^values -> :ok end)

      message = {:sync_write, :goal_position, values}
      assert {:noreply, ^state} = Controller.handle_cast(message, state)
    end
  end
end
