# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.ActuatorTest do
  use ExUnit.Case, async: false
  use Mimic

  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.Feetech.Actuator
  alias BB.Servo.Feetech.Actuator.State

  setup :verify_on_exit!

  @pi :math.pi()

  describe "disarm/1" do
    test "returns :ok (torque managed by controller)" do
      assert :ok = Actuator.disarm([])
    end
  end

  describe "init/1" do
    setup do
      servo_table = :ets.new(:test_init_table, [:set, :public])

      joint = %{
        type: :revolute,
        limits: %{
          lower: -@pi / 2,
          upper: @pi / 2,
          velocity: @pi / 3
        }
      }

      BB.Robot
      |> stub(:get_joint, fn _robot, :shoulder -> joint end)

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:register_servo, _, _, _, _, _} -> {:ok, servo_table}
        end
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      %{joint: joint, servo_table: servo_table}
    end

    test "initialises with valid revolute joint", %{servo_table: servo_table} do
      :ets.insert(
        servo_table,
        {1, :shoulder, 0.0, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
      )

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, state} = Actuator.init(opts)

      assert state.servo_id == 1
      assert state.controller == :feetech
      assert state.name == :servo
      assert state.joint_name == :shoulder
      assert state.servo_table == servo_table
      assert_in_delta state.lower_limit, -@pi / 2, 0.001
      assert_in_delta state.upper_limit, @pi / 2, 0.001
      assert_in_delta state.center_angle, 0.0, 0.001
      assert_in_delta state.range, @pi, 0.001
      assert is_nil(state.trajectory)
      assert is_nil(state.trajectory_timer)
    end

    test "disables torque on init" do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> stub(:call, fn _robot, _controller, _msg -> {:ok, :stub_table} end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "registers servo with controller and receives table ref", %{servo_table: servo_table} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> expect(:call, fn TestRobot,
                          :feetech,
                          {:register_servo, 1, :shoulder, center, 2, false} ->
        assert_in_delta center, 0.0, 0.001
        {:ok, servo_table}
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.servo_table == servo_table
    end

    test "subscribes to position commands" do
      BB
      |> expect(:subscribe, fn TestRobot, [:actuator, :shoulder, :servo] -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "fails for continuous joints" do
      continuous_joint = %{
        type: :continuous,
        limits: nil
      }

      BB.Robot
      |> expect(:get_joint, fn _robot, :wheel -> continuous_joint end)

      opts = [
        bb: %{robot: TestRobot, path: [:wheel, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{}} = Actuator.init(opts)
    end

    test "fails when joint has no limits" do
      no_limits_joint = %{
        type: :revolute,
        limits: nil
      }

      BB.Robot
      |> expect(:get_joint, fn _robot, :joint -> no_limits_joint end)

      opts = [
        bb: %{robot: TestRobot, path: [:joint, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{}} = Actuator.init(opts)
    end

    test "fails when joint not found" do
      BB.Robot
      |> expect(:get_joint, fn _robot, :missing -> nil end)

      opts = [
        bb: %{robot: TestRobot, path: [:missing, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{}} = Actuator.init(opts)
    end
  end

  describe "position conversion" do
    setup :armed_state

    test "converts center angle to servo center (2048)", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: 0.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, goal_speed}] = :ets.lookup(servo_table, 1)
      assert goal_position == 2048
      assert goal_speed == 0
    end

    test "converts positive angle to higher servo position", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: @pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # pi/4 radians = 45 degrees = 512 steps from center
      # 2048 + 512 = 2560
      assert_in_delta goal_position, 2560, 1
    end

    test "converts negative angle to lower servo position", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: -@pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # -pi/4 radians = -45 degrees = -512 steps from center
      # 2048 - 512 = 1536
      assert_in_delta goal_position, 1536, 1
    end

    test "reverses direction when reverse? is true", %{state: state, servo_table: servo_table} do
      state = %{state | reverse?: true}

      cmd = %Command.Position{position: @pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # With reverse, positive angle should give lower position
      # pi/4 should give 2048 - 512 = 1536
      assert_in_delta goal_position, 1536, 1
    end
  end

  describe "position clamping" do
    setup :armed_state

    test "clamps position above upper limit", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: @pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert_in_delta new_state.current_angle, @pi / 2, 0.001

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # Position should be clamped to upper limit (pi/2)
      # pi/2 = 1024 steps from center = 2048 + 1024 = 3072
      assert_in_delta goal_position, 3072, 1
    end

    test "clamps position below lower limit", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: -@pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert_in_delta new_state.current_angle, -@pi / 2, 0.001

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # Position should be clamped to lower limit (-pi/2)
      # -pi/2 = -1024 steps from center = 2048 - 1024 = 1024
      assert_in_delta goal_position, 1024, 1
    end
  end

  describe "command handling when not armed" do
    setup :unarmed_state

    test "ignores pubsub position commands when not armed", %{state: state} do
      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, ^state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "ignores pubsub trajectory commands when not armed", %{state: state} do
      cmd = %Command.Trajectory{
        waypoints: [[position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]]
      }

      msg = %Message{payload: cmd}

      assert {:noreply, ^state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "ignores cast commands when not armed", %{state: state} do
      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, ^state} = Actuator.handle_cast({:command, msg}, state)
    end

    test "returns error for sync commands when not armed", %{state: state} do
      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:reply, {:error, :not_armed}, ^state} =
               Actuator.handle_call({:command, msg}, {self(), make_ref()}, state)
    end
  end

  describe "BeginMotion publishing" do
    setup :armed_state

    test "publishes BeginMotion message after command", %{state: state} do
      BB
      |> expect(:publish, fn TestRobot, [:actuator, :shoulder, :servo], msg ->
        assert msg.payload.__struct__ == BB.Message.Actuator.BeginMotion
        assert_in_delta msg.payload.initial_position, 0.0, 0.001
        assert_in_delta msg.payload.target_position, 0.5, 0.001
        assert msg.payload.command_type == :position
        :ok
      end)

      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "includes command_id when provided", %{state: state} do
      command_id = make_ref()

      BB
      |> expect(:publish, fn TestRobot, [:actuator, :shoulder, :servo], msg ->
        assert msg.payload.command_id == command_id
        :ok
      end)

      cmd = %Command.Position{position: 0.5, command_id: command_id}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end
  end

  describe "velocity hints" do
    setup :armed_state

    test "writes goal_speed to ETS when velocity provided", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5, velocity: 1.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, _, goal_speed}] = :ets.lookup(servo_table, 1)
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "uses velocity for expected_arrival calculation", %{state: state} do
      BB
      |> expect(:publish, fn TestRobot, [:actuator, :shoulder, :servo], msg ->
        # Moving 0.5 rad at 1.0 rad/s = 500ms
        now = System.monotonic_time(:millisecond)
        expected_travel = round(0.5 / 1.0 * 1000)
        assert_in_delta msg.payload.expected_arrival, now + expected_travel, 50
        :ok
      end)

      cmd = %Command.Position{position: 0.5, velocity: 1.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "resets goal_speed to 0 when no hints provided", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, _, goal_speed}] = :ets.lookup(servo_table, 1)
      assert goal_speed == 0
    end
  end

  describe "duration hints" do
    setup :armed_state

    test "computes velocity from distance and duration, writes goal_speed", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5, duration: 500}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, _, goal_speed}] = :ets.lookup(servo_table, 1)
      # Moving 0.5 rad in 500ms → velocity = 0.5 / 0.5 = 1.0 rad/s
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "uses duration for expected_arrival calculation", %{state: state} do
      BB
      |> expect(:publish, fn TestRobot, [:actuator, :shoulder, :servo], msg ->
        now = System.monotonic_time(:millisecond)
        assert_in_delta msg.payload.expected_arrival, now + 500, 50
        :ok
      end)

      cmd = %Command.Position{position: 0.5, duration: 500}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "velocity hint takes precedence over duration hint", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5, velocity: 2.0, duration: 5000}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      [{1, _, _, _, _, _, _, _, _, _, _, _, goal_speed}] = :ets.lookup(servo_table, 1)
      assert_in_delta goal_speed, 2.0, 0.001
    end
  end

  describe "trajectory command handling" do
    setup :armed_state

    test "executes first waypoint immediately", %{state: state, servo_table: servo_table} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0],
          [position: 1.0, velocity: 0.5, acceleration: 0.0, time_from_start: 500]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert new_state.trajectory != nil
      assert new_state.trajectory.index == 0
      assert_in_delta new_state.current_angle, 0.5, 0.001

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, goal_speed}] = :ets.lookup(servo_table, 1)
      assert goal_position != nil
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "publishes BeginMotion with command_type :trajectory", %{state: state} do
      BB
      |> expect(:publish, fn TestRobot, [:actuator, :shoulder, :servo], msg ->
        assert msg.payload.command_type == :trajectory
        assert_in_delta msg.payload.initial_position, 0.0, 0.001
        assert_in_delta msg.payload.target_position, 0.5, 0.001
        :ok
      end)

      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "clamps waypoint positions to joint limits", %{state: state, servo_table: servo_table} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: @pi, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert_in_delta new_state.current_angle, @pi / 2, 0.001

      [{1, _, _, _, _, _, _, _, _, _, _, goal_position, _}] = :ets.lookup(servo_table, 1)
      # pi should be clamped to pi/2 = 3072
      assert_in_delta goal_position, 3072, 1
    end

    test "single waypoint trajectory completes after advancement", %{state: state} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, state_after_start} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert state_after_start.trajectory != nil

      {:noreply, state_after_advance} =
        Actuator.handle_info(:trajectory_next, state_after_start)

      assert is_nil(state_after_advance.trajectory)
      assert is_nil(state_after_advance.trajectory_timer)
    end

    test "advances through multi-waypoint trajectory", %{state: state} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.2, velocity: 1.0, acceleration: 0.0, time_from_start: 0],
          [position: 0.4, velocity: 0.5, acceleration: 0.0, time_from_start: 200],
          [position: 0.6, velocity: 0.0, acceleration: 0.0, time_from_start: 400]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert state.trajectory.index == 0
      assert_in_delta state.current_angle, 0.2, 0.001

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory.index == 1
      assert_in_delta state.current_angle, 0.4, 0.001

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory.index == 2
      assert_in_delta state.current_angle, 0.6, 0.001

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert is_nil(state.trajectory)
    end
  end

  describe "trajectory cancellation" do
    setup :armed_state

    test "position command cancels active trajectory", %{state: state} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.2, velocity: 1.0, acceleration: 0.0, time_from_start: 0],
          [position: 0.4, velocity: 0.5, acceleration: 0.0, time_from_start: 500]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, state_with_trajectory} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert state_with_trajectory.trajectory != nil

      pos_cmd = %Command.Position{position: 0.0}
      pos_msg = %Message{payload: pos_cmd}

      {:noreply, state_after_position} =
        Actuator.handle_info(
          {:bb, [:actuator, :shoulder, :servo], pos_msg},
          state_with_trajectory
        )

      assert is_nil(state_after_position.trajectory)
      assert is_nil(state_after_position.trajectory_timer)
    end

    test "new trajectory cancels active trajectory", %{state: state} do
      first_cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.2, velocity: 1.0, acceleration: 0.0, time_from_start: 0],
          [position: 0.4, velocity: 0.5, acceleration: 0.0, time_from_start: 500]
        ]
      }

      {:noreply, state_with_first} =
        Actuator.handle_info(
          {:bb, [:actuator, :shoulder, :servo], %Message{payload: first_cmd}},
          state
        )

      second_cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.6, velocity: 2.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      {:noreply, state_with_second} =
        Actuator.handle_info(
          {:bb, [:actuator, :shoulder, :servo], %Message{payload: second_cmd}},
          state_with_first
        )

      assert state_with_second.trajectory != nil
      assert_in_delta state_with_second.current_angle, 0.6, 0.001
    end
  end

  describe "trajectory repeat" do
    setup :armed_state

    test "repeats trajectory when repeat > 1", %{state: state} do
      cmd = %Command.Trajectory{
        repeat: 2,
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert state.trajectory.repeat == 2

      # First completion → restarts with repeat=1
      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory != nil
      assert state.trajectory.repeat == 1
      assert state.trajectory.index == 0

      # Second completion → done
      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert is_nil(state.trajectory)
    end

    test "repeats forever when repeat is :forever", %{state: state} do
      cmd = %Command.Trajectory{
        repeat: :forever,
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      assert state.trajectory.repeat == :forever

      # Keeps repeating
      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory != nil
      assert state.trajectory.repeat == :forever

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory != nil
      assert state.trajectory.repeat == :forever
    end
  end

  # --- Test helpers ---

  defp armed_state(_context) do
    servo_table = :ets.new(:test_servo_table, [:set, :public])

    :ets.insert(
      servo_table,
      {1, :shoulder, 0.0, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
    )

    state = %State{
      bb: %{robot: TestRobot, path: [:shoulder, :servo]},
      center_angle: 0.0,
      controller: :feetech,
      current_angle: 0.0,
      joint_name: :shoulder,
      lower_limit: -@pi / 2,
      name: :servo,
      range: @pi,
      servo_id: 1,
      servo_table: servo_table,
      upper_limit: @pi / 2,
      velocity_limit: @pi / 3
    }

    BB.Safety
    |> stub(:armed?, fn _robot -> true end)

    BB
    |> stub(:publish, fn _robot, _path, _msg -> :ok end)

    on_exit(fn ->
      if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
    end)

    %{state: state, servo_table: servo_table}
  end

  defp unarmed_state(_context) do
    servo_table = :ets.new(:test_servo_table, [:set, :public])

    :ets.insert(
      servo_table,
      {1, :shoulder, 0.0, false, 2, nil, nil, nil, nil, nil, nil, nil, nil}
    )

    state = %State{
      bb: %{robot: TestRobot, path: [:shoulder, :servo]},
      center_angle: 0.0,
      controller: :feetech,
      current_angle: 0.0,
      joint_name: :shoulder,
      lower_limit: -@pi / 2,
      name: :servo,
      range: @pi,
      servo_id: 1,
      servo_table: servo_table,
      upper_limit: @pi / 2,
      velocity_limit: @pi / 3
    }

    BB.Safety
    |> stub(:armed?, fn _robot -> false end)

    on_exit(fn ->
      if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
    end)

    %{state: state, servo_table: servo_table}
  end
end
