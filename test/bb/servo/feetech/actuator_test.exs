# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.ActuatorTest do
  use ExUnit.Case, async: false
  use Mimic

  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.Feetech.Actuator

  setup :verify_on_exit!

  @pi :math.pi()

  describe "disarm/1" do
    test "returns :ok (torque managed by controller)" do
      assert :ok = Actuator.disarm([])
    end
  end

  describe "init/1" do
    setup do
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
          {:write_raw, _id, :goal_position, _pos} -> :ok
          {:register_servo, _, _, _, _, _} -> :ok
        end
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      %{joint: joint}
    end

    test "initialises with valid revolute joint" do
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
      assert_in_delta state.lower_limit, -@pi / 2, 0.001
      assert_in_delta state.upper_limit, @pi / 2, 0.001
      assert_in_delta state.center_angle, 0.0, 0.001
      assert_in_delta state.range, @pi, 0.001
    end

    test "disables torque on init" do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> stub(:call, fn _robot, _controller, _msg -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "sets initial position to joint center" do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> expect(:call, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        assert position == 2048
        :ok
      end)
      |> expect(:call, fn TestRobot, :feetech, {:register_servo, _, _, _, _, _} -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "registers servo with controller" do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> expect(:call, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, _} -> :ok end)
      |> expect(:call, fn TestRobot,
                          :feetech,
                          {:register_servo, 1, :shoulder, center, 2, false} ->
        assert_in_delta center, 0.0, 0.001
        :ok
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech
      ]

      assert {:ok, _state} = Actuator.init(opts)
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
    setup do
      state = %{
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        reverse?: false,
        position_deadband: 2,
        lower_limit: -@pi / 2,
        upper_limit: @pi / 2,
        center_angle: 0.0,
        range: @pi,
        velocity_limit: @pi / 3,
        current_angle: 0.0,
        name: :servo,
        joint_name: :shoulder
      }

      BB.Safety
      |> stub(:armed?, fn _robot -> true end)

      BB.Process
      |> stub(:cast, fn _robot, _controller, _msg -> :ok end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      %{state: state}
    end

    test "converts center angle to servo center (2048)", %{state: state} do
      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        assert position == 2048
        :ok
      end)

      cmd = %Command.Position{position: 0.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "converts positive angle to higher servo position", %{state: state} do
      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        # pi/4 radians = 45 degrees = 512 steps from center
        # 2048 + 512 = 2560
        assert_in_delta position, 2560, 1
        :ok
      end)

      cmd = %Command.Position{position: @pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "converts negative angle to lower servo position", %{state: state} do
      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        # -pi/4 radians = -45 degrees = -512 steps from center
        # 2048 - 512 = 1536
        assert_in_delta position, 1536, 1
        :ok
      end)

      cmd = %Command.Position{position: -@pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "reverses direction when reverse? is true", %{state: state} do
      state = %{state | reverse?: true}

      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        # With reverse, positive angle should give lower position
        # pi/4 should give 2048 - 512 = 1536
        assert_in_delta position, 1536, 1
        :ok
      end)

      cmd = %Command.Position{position: @pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end
  end

  describe "position clamping" do
    setup do
      state = %{
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        reverse?: false,
        position_deadband: 2,
        lower_limit: -@pi / 2,
        upper_limit: @pi / 2,
        center_angle: 0.0,
        range: @pi,
        velocity_limit: @pi / 3,
        current_angle: 0.0,
        name: :servo,
        joint_name: :shoulder
      }

      BB.Safety
      |> stub(:armed?, fn _robot -> true end)

      BB.Process
      |> stub(:cast, fn _robot, _controller, _msg -> :ok end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      %{state: state}
    end

    test "clamps position above upper limit", %{state: state} do
      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        # Position should be clamped to upper limit (pi/2)
        # pi/2 = 1024 steps from center = 2048 + 1024 = 3072
        assert_in_delta position, 3072, 1
        :ok
      end)

      # Request position way above limit
      cmd = %Command.Position{position: @pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      # State should reflect clamped value
      assert_in_delta new_state.current_angle, @pi / 2, 0.001
    end

    test "clamps position below lower limit", %{state: state} do
      BB.Process
      |> expect(:cast, fn TestRobot, :feetech, {:write_raw, 1, :goal_position, position} ->
        # Position should be clamped to lower limit (-pi/2)
        # -pi/2 = -1024 steps from center = 2048 - 1024 = 1024
        assert_in_delta position, 1024, 1
        :ok
      end)

      # Request position way below limit
      cmd = %Command.Position{position: -@pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)

      # State should reflect clamped value
      assert_in_delta new_state.current_angle, -@pi / 2, 0.001
    end
  end

  describe "command handling when not armed" do
    setup do
      state = %{
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        reverse?: false,
        position_deadband: 2,
        lower_limit: -@pi / 2,
        upper_limit: @pi / 2,
        center_angle: 0.0,
        range: @pi,
        velocity_limit: @pi / 3,
        current_angle: 0.0,
        name: :servo,
        joint_name: :shoulder
      }

      %{state: state}
    end

    test "ignores pubsub commands when not armed", %{state: state} do
      BB.Safety
      |> expect(:armed?, fn TestRobot -> false end)

      # Should not call BB.Process.cast
      BB.Process
      |> reject(:cast, 3)

      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, ^state} =
               Actuator.handle_info({:bb, [:actuator, :shoulder, :servo], msg}, state)
    end

    test "ignores cast commands when not armed", %{state: state} do
      BB.Safety
      |> expect(:armed?, fn TestRobot -> false end)

      BB.Process
      |> reject(:cast, 3)

      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, ^state} = Actuator.handle_cast({:command, msg}, state)
    end

    test "returns error for sync commands when not armed", %{state: state} do
      BB.Safety
      |> expect(:armed?, fn TestRobot -> false end)

      BB.Process
      |> reject(:cast, 3)

      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:reply, {:error, :not_armed}, ^state} =
               Actuator.handle_call({:command, msg}, {self(), make_ref()}, state)
    end
  end

  describe "BeginMotion publishing" do
    setup do
      state = %{
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        reverse?: false,
        position_deadband: 2,
        lower_limit: -@pi / 2,
        upper_limit: @pi / 2,
        center_angle: 0.0,
        range: @pi,
        velocity_limit: @pi / 3,
        current_angle: 0.0,
        name: :servo,
        joint_name: :shoulder
      }

      BB.Safety
      |> stub(:armed?, fn _robot -> true end)

      BB.Process
      |> stub(:cast, fn _robot, _controller, _msg -> :ok end)

      %{state: state}
    end

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
end
