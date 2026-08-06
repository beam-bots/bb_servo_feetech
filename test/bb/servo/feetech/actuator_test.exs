# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.ActuatorTest do
  use ExUnit.Case, async: false
  use Mimic

  import BB.Unit

  alias BB.Actuator.MotorProfile
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.Feetech.Actuator
  alias BB.Servo.Feetech.Actuator.State

  setup :verify_on_exit!

  @pi :math.pi()

  defp motor_profile(overrides \\ []) do
    base = %MotorProfile{
      motor_lower: -@pi / 2,
      motor_upper: @pi / 2,
      motor_velocity_limit: @pi / 3,
      motor_initial_position: 0.0
    }

    struct!(base, overrides)
  end

  describe "disarm/1" do
    test "returns :ok (torque managed by controller)" do
      assert :ok = Actuator.disarm([])
    end
  end

  describe "init/1" do
    setup do
      servo_table = :ets.new(:test_init_table, [:set, :public])

      BB.Dsl.Info
      |> stub(:controllers, fn TestRobot -> [%BB.Dsl.Controller{name: :feetech}] end)

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:write, _id, :acceleration, _} -> :ok
          {:register_servo, _, _, _} -> {:ok, servo_table}
        end
      end)

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      %{servo_table: servo_table}
    end

    test "initialises with a complete motor profile", %{servo_table: servo_table} do
      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile()
      ]

      assert {:ok, state} = Actuator.init(opts)

      assert state.servo_id == 1
      assert state.controller == :feetech
      assert state.name == :servo
      assert state.joint_name == :shoulder
      assert state.servo_table == servo_table
      assert_in_delta state.motor_profile.motor_lower, -@pi / 2, 0.001
      assert_in_delta state.motor_profile.motor_upper, @pi / 2, 0.001
      assert_in_delta state.motor_profile.motor_velocity_limit, @pi / 3, 0.001
      assert is_nil(state.trajectory)
      assert is_nil(state.trajectory_timer)
    end

    test "disables torque on init", %{servo_table: servo_table} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:write, _id, :acceleration, _} -> :ok
          {:register_servo, _, _, _} -> {:ok, servo_table}
        end
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile()
      ]

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "registers with controller, passing its own actuator path", %{servo_table: servo_table} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)
      |> expect(:call, fn TestRobot, :feetech, {:read, 1, :model_number} -> {:ok, 777} end)
      |> expect(:call, fn TestRobot, :feetech, {:read, 1, :mode} -> {:ok, :position} end)
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :acceleration, 0} -> :ok end)
      |> expect(:call, fn TestRobot, :feetech, {:register_servo, 1, [:shoulder, :servo], 2} ->
        {:ok, servo_table}
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile()
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.servo_table == servo_table
    end

    test "fails when motor profile has no lower limit" do
      opts = [
        bb: %{robot: TestRobot, path: [:wheel, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile(motor_lower: nil)
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :lower}} = Actuator.init(opts)
    end

    test "fails when motor profile has no upper limit" do
      opts = [
        bb: %{robot: TestRobot, path: [:joint, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile(motor_upper: nil)
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :upper}} = Actuator.init(opts)
    end

    test "fails when motor profile has no velocity limit" do
      opts = [
        bb: %{robot: TestRobot, path: [:joint, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile(motor_velocity_limit: nil)
      ]

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :velocity}} = Actuator.init(opts)
    end

    test "writes the joint's acceleration limit to the servo" do
      test_pid = self()

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:register_servo, _, _, _} -> {:ok, :table}
          {:write, _id, :acceleration, raw} -> send(test_pid, {:acceleration, raw}) && :ok
        end
      end)

      # The SO-101 declares 2160 degree_per_square_second, which is 37.7 rad/s²
      # against a register counting hundreds of steps per second squared.
      opts =
        base_opts(motor_profile: motor_profile(motor_acceleration_limit: 2160 * @pi / 180))

      assert {:ok, _state} = Actuator.init(opts)
      assert_received {:acceleration, 246}
    end

    test "writes 0 when the joint declares no acceleration limit" do
      test_pid = self()

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:register_servo, _, _, _} -> {:ok, :table}
          {:write, _id, :acceleration, raw} -> send(test_pid, {:acceleration, raw}) && :ok
        end
      end)

      assert {:ok, _state} = Actuator.init(base_opts())
      assert_received {:acceleration, 0}
    end

    test "rounds a tiny acceleration limit up rather than down to no limit" do
      test_pid = self()

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:register_servo, _, _, _} -> {:ok, :table}
          {:write, _id, :acceleration, raw} -> send(test_pid, {:acceleration, raw}) && :ok
        end
      end)

      opts = base_opts(motor_profile: motor_profile(motor_acceleration_limit: 0.001))

      assert {:ok, _state} = Actuator.init(opts)
      # 0 would mean "no limit" — the opposite of a very gentle one.
      assert_received {:acceleration, 1}
    end

    test "refuses a joint accelerating harder than the register can express" do
      opts = base_opts(motor_profile: motor_profile(motor_acceleration_limit: 100.0))

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :acceleration} = error} =
               Actuator.init(opts)

      assert error.message =~ "beyond the"
    end

    test "refuses to start when :controller doesn't name a declared controller" do
      BB.Dsl.Info
      |> stub(:controllers, fn TestRobot ->
        [%BB.Dsl.Controller{name: :dynamixel}, %BB.Dsl.Controller{name: :pca9685}]
      end)

      assert {:stop, %BB.Error.Invalid.Feetech.UnknownController{} = error} =
               Actuator.init(base_opts())

      assert error.controller == :feetech
      assert error.actuator_path == [:shoulder, :servo]
      assert error.known == [:dynamixel, :pca9685]

      assert Exception.message(error) =~ "It has: :dynamixel, :pca9685"
    end

    test "says so plainly when the robot declares no controllers at all" do
      BB.Dsl.Info |> stub(:controllers, fn TestRobot -> [] end)

      assert {:stop, %BB.Error.Invalid.Feetech.UnknownController{known: []} = error} =
               Actuator.init(base_opts())

      assert Exception.message(error) =~ "no controllers at all"
    end

    test "checks the controller before touching the bus" do
      BB.Dsl.Info |> stub(:controllers, fn TestRobot -> [] end)

      BB.Process |> reject(:call, 3)

      assert {:stop, %BB.Error.Invalid.Feetech.UnknownController{}} = Actuator.init(base_opts())
    end

    test "refuses to start when the controller says the servo ID is taken" do
      taken = %BB.Error.Invalid.Feetech.DuplicateServoId{
        servo_id: 1,
        actuator_path: [:shoulder, :servo],
        registered_path: [:elbow, :servo]
      }

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:write, _id, :acceleration, _} -> :ok
          {:register_servo, _, _, _} -> {:error, taken}
        end
      end)

      assert {:stop, ^taken} = Actuator.init(base_opts())
    end

    test "refuses a joint faster than the speed register can express" do
      # The SO-101 declares 360 degree_per_second, which is exactly 4096
      # steps/s — one past the end of a 0..4095 register.
      opts = base_opts(motor_profile: motor_profile(motor_velocity_limit: 2 * @pi))

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :velocity} = error} =
               Actuator.init(opts)

      assert error.message =~ "faster than"
    end

    test "accepts a joint at exactly the top of the speed register" do
      limit = 4095 * (2 * @pi / 4096)
      opts = base_opts(motor_profile: motor_profile(motor_velocity_limit: limit))

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "refuses a joint that reaches past the encoder" do
      opts = base_opts(motor_profile: motor_profile(motor_upper: @pi))

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :upper} = error} = Actuator.init(opts)
      assert error.message =~ "beyond the"
    end

    test "refuses a joint that reaches below the encoder" do
      opts = base_opts(motor_profile: motor_profile(motor_lower: -@pi - 0.1))

      assert {:stop, %BB.Error.Invalid.JointConfig{field: :lower} = error} = Actuator.init(opts)
      assert error.message =~ "below the"
    end

    test "accepts a joint spanning the full encoder" do
      opts =
        base_opts(
          motor_profile: motor_profile(motor_lower: -@pi, motor_upper: 2047 * (2 * @pi / 4096))
        )

      assert {:ok, _state} = Actuator.init(opts)
    end

    test "takes the stall torque from the model the servo reports" do
      assert {:ok, state} = Actuator.init(base_opts())
      assert_in_delta state.stall_torque, 1.912, 0.001
    end

    test "prefers a configured stall torque over the model's" do
      assert {:ok, state} = Actuator.init(base_opts(stall_torque: ~u(3.5 newton_meter)))
      assert_in_delta state.stall_torque, 3.5, 0.001
    end

    test "refuses to start when the model is unknown and no stall torque is given" do
      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 4242}
        end
      end)

      assert {:stop, %BB.Error.Invalid.Feetech.StallTorque{model_number: 4242, servo_id: 1}} =
               Actuator.init(base_opts())
    end

    test "starts an unknown model when the stall torque is given", %{servo_table: servo_table} do
      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :mode} -> {:ok, :position}
          {:write, _id, :acceleration, _} -> :ok
          {:register_servo, _, _, _} -> {:ok, servo_table}
        end
      end)

      assert {:ok, state} = Actuator.init(base_opts(stall_torque: ~u(3.5 newton_meter)))
      assert_in_delta state.stall_torque, 3.5, 0.001
    end

    test "leaves the mode alone when the servo is already in it" do
      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:write, _id, :mode, _} -> flunk("rewrote a mode the servo was already in")
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :velocity}
          {:write, _id, :acceleration, _} -> :ok
          {:register_servo, _, _, _} -> {:ok, :table}
        end
      end)

      assert {:ok, state} = Actuator.init(base_opts(mode: :velocity))
      assert state.mode == :velocity
    end

    test "unlocks the EEPROM around a mode change, and locks it again" do
      test_pid = self()

      BB.Process
      |> stub(:call, fn _robot, _controller, msg ->
        case msg do
          {:write, _id, :torque_enable, false} -> :ok
          {:read, _id, :model_number} -> {:ok, 777}
          {:read, _id, :mode} -> {:ok, :position}
          {:register_servo, _, _, _} -> {:ok, :table}
          {:write, _id, param, value} -> send(test_pid, {param, value}) && :ok
        end
      end)

      assert {:ok, _state} = Actuator.init(base_opts(mode: :velocity))

      eeprom = Enum.filter(drain_messages(), fn {param, _} -> param in [:lock, :mode] end)
      assert eeprom == [{:lock, false}, {:mode, :velocity}, {:lock, true}]
    end
  end

  describe "position conversion" do
    setup :armed_state

    test "converts center angle to servo center (2048)", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: 0.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_command(msg, state)

      goal_position = goal(servo_table, :goal_position)
      goal_speed = goal(servo_table, :goal_speed)
      assert goal_position == 2048
      assert_in_delta goal_speed, @pi / 3, 0.001
    end

    test "converts positive angle to higher servo position", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: @pi / 4}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_command(msg, state)

      goal_position = goal(servo_table, :goal_position)
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
               Actuator.handle_command(msg, state)

      goal_position = goal(servo_table, :goal_position)
      # -pi/4 radians = -45 degrees = -512 steps from center
      # 2048 - 512 = 1536
      assert_in_delta goal_position, 1536, 1
    end
  end

  describe "position clamping" do
    setup :armed_state

    test "clamps position above upper limit", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: @pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_command(msg, state)

      assert_in_delta new_state.current_motor_angle, @pi / 2, 0.001

      goal_position = goal(servo_table, :goal_position)
      # Position should be clamped to upper limit (pi/2)
      # pi/2 = 1024 steps from center = 2048 + 1024 = 3072
      assert_in_delta goal_position, 3072, 1
    end

    test "clamps position below lower limit", %{state: state, servo_table: servo_table} do
      cmd = %Command.Position{position: -@pi}
      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_command(msg, state)

      assert_in_delta new_state.current_motor_angle, -@pi / 2, 0.001

      goal_position = goal(servo_table, :goal_position)
      # Position should be clamped to lower limit (-pi/2)
      # -pi/2 = -1024 steps from center = 2048 - 1024 = 1024
      assert_in_delta goal_position, 1024, 1
    end
  end

  describe "BeginMotion publishing" do
    setup :armed_state

    test "publishes BeginMotion message after command", %{state: state} do
      BB.Actuator
      |> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :servo], opts ->
        assert_in_delta opts[:initial_position], 0.0, 0.001
        assert_in_delta opts[:target_position], 0.5, 0.001
        assert opts[:command_type] == :position
        :ok
      end)

      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_command(msg, state)
    end

    test "includes command_id when provided", %{state: state} do
      command_id = make_ref()

      BB.Actuator
      |> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :servo], opts ->
        assert opts[:command_id] == command_id
        :ok
      end)

      cmd = %Command.Position{position: 0.5, command_id: command_id}
      msg = %Message{payload: cmd}

      assert {:noreply, _new_state} =
               Actuator.handle_command(msg, state)
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
               Actuator.handle_command(msg, state)

      goal_speed = goal(servo_table, :goal_speed)
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "uses velocity for expected_arrival calculation", %{state: state} do
      BB.Actuator
      |> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :servo], opts ->
        # Moving 0.5 rad at 1.0 rad/s = 500ms
        now = System.monotonic_time(:millisecond)
        expected_travel = round(0.5 / 1.0 * 1000)
        assert_in_delta opts[:expected_arrival], now + expected_travel, 50
        :ok
      end)

      cmd = %Command.Position{position: 0.5, velocity: 1.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)
    end

    test "falls back to the joint's velocity limit when no hints provided", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)

      assert_in_delta goal(servo_table, :goal_speed), @pi / 3, 0.001
    end

    test "clamps a velocity hint above the joint's limit", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5, velocity: 100.0}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)

      assert_in_delta goal(servo_table, :goal_speed), @pi / 3, 0.001
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
               Actuator.handle_command(msg, state)

      goal_speed = goal(servo_table, :goal_speed)
      # Moving 0.5 rad in 500ms → velocity = 0.5 / 0.5 = 1.0 rad/s
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "uses duration for expected_arrival calculation", %{state: state} do
      BB.Actuator
      |> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :servo], opts ->
        now = System.monotonic_time(:millisecond)
        assert_in_delta opts[:expected_arrival], now + 500, 50
        :ok
      end)

      cmd = %Command.Position{position: 0.5, duration: 500}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)
    end

    test "velocity hint takes precedence over duration hint", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Position{position: 0.5, velocity: 0.8, duration: 5000}
      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)

      goal_speed = goal(servo_table, :goal_speed)
      assert_in_delta goal_speed, 0.8, 0.001
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
        Actuator.handle_command(msg, state)

      assert new_state.trajectory != nil
      assert new_state.trajectory.index == 0
      assert_in_delta new_state.current_motor_angle, 0.5, 0.001

      goal_position = goal(servo_table, :goal_position)
      goal_speed = goal(servo_table, :goal_speed)
      assert goal_position != nil
      assert_in_delta goal_speed, 1.0, 0.001
    end

    test "publishes BeginMotion with command_type :trajectory", %{state: state} do
      BB.Actuator
      |> expect(:publish_begin_motion, fn TestRobot, [:shoulder, :servo], opts ->
        assert opts[:command_type] == :trajectory
        assert_in_delta opts[:initial_position], 0.0, 0.001
        assert_in_delta opts[:target_position], 0.5, 0.001
        :ok
      end)

      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.5, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      assert {:noreply, _state} =
               Actuator.handle_command(msg, state)
    end

    test "clamps waypoint positions to joint limits", %{state: state, servo_table: servo_table} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: @pi, velocity: 1.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      msg = %Message{payload: cmd}

      {:noreply, new_state} =
        Actuator.handle_command(msg, state)

      assert_in_delta new_state.current_motor_angle, @pi / 2, 0.001

      goal_position = goal(servo_table, :goal_position)
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
        Actuator.handle_command(msg, state)

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
        Actuator.handle_command(msg, state)

      assert state.trajectory.index == 0
      assert_in_delta state.current_motor_angle, 0.2, 0.001

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory.index == 1
      assert_in_delta state.current_motor_angle, 0.4, 0.001

      {:noreply, state} = Actuator.handle_info(:trajectory_next, state)
      assert state.trajectory.index == 2
      assert_in_delta state.current_motor_angle, 0.6, 0.001

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
        Actuator.handle_command(msg, state)

      assert state_with_trajectory.trajectory != nil

      pos_cmd = %Command.Position{position: 0.0}
      pos_msg = %Message{payload: pos_cmd}

      {:noreply, state_after_position} =
        Actuator.handle_command(pos_msg, state_with_trajectory)

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
        Actuator.handle_command(%Message{payload: first_cmd}, state)

      second_cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.6, velocity: 2.0, acceleration: 0.0, time_from_start: 0]
        ]
      }

      {:noreply, state_with_second} =
        Actuator.handle_command(%Message{payload: second_cmd}, state_with_first)

      assert state_with_second.trajectory != nil
      assert_in_delta state_with_second.current_motor_angle, 0.6, 0.001
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
        Actuator.handle_command(msg, state)

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
        Actuator.handle_command(msg, state)

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

  describe "command_payloads/1" do
    test "position mode admits position, trajectory, effort, hold and stop" do
      assert Actuator.command_payloads(mode: :position) ==
               Enum.sort([
                 Command.Effort,
                 Command.Hold,
                 Command.Position,
                 Command.Stop,
                 Command.Trajectory
               ])
    end

    test "velocity mode admits velocity, effort, hold and stop" do
      assert Actuator.command_payloads(mode: :velocity) ==
               Enum.sort([Command.Effort, Command.Hold, Command.Stop, Command.Velocity])
    end

    test "defaults to position mode" do
      assert Actuator.command_payloads([]) == Actuator.command_payloads(mode: :position)
    end

    test "position mode does not admit velocity" do
      refute Command.Velocity in Actuator.command_payloads(mode: :position)
    end

    test "velocity mode does not admit position or trajectory" do
      payloads = Actuator.command_payloads(mode: :velocity)
      refute Command.Position in payloads
      refute Command.Trajectory in payloads
    end
  end

  describe "Command.Stop" do
    setup :armed_state

    test "cuts torque and abandons the pending goal", %{
      state: state,
      servo_table: servo_table
    } do
      :ets.update_element(servo_table, 1, [{10, [goal_position: 3000]}])

      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Stop{}}, state)

      assert pending_writes(servo_table) == nil
    end

    test "decelerate stops the same way immediate does", %{state: state} do
      BB.Process
      |> stub(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)

      assert {:noreply, _state} =
               Actuator.handle_command(
                 %Message{payload: %Command.Stop{mode: :decelerate}},
                 state
               )
    end

    test "cancels a running trajectory", %{state: state} do
      cmd = %Command.Trajectory{
        waypoints: [
          [position: 0.2, velocity: 1.0, acceleration: 0.0, time_from_start: 0],
          [position: 0.4, velocity: 0.5, acceleration: 0.0, time_from_start: 500]
        ]
      }

      {:noreply, state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert state.trajectory != nil

      BB.Process
      |> stub(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)

      {:noreply, state} = Actuator.handle_command(%Message{payload: %Command.Stop{}}, state)

      assert is_nil(state.trajectory)
      assert is_nil(state.trajectory_timer)
    end
  end

  describe "Command.Hold" do
    test "does nothing to a powered servo in position mode", context do
      %{state: state} = armed_state(context)

      BB.Process
      |> reject(:call, 3)

      assert {:noreply, ^state} =
               Actuator.handle_command(%Message{payload: %Command.Hold{}}, state)
    end

    test "commands a standstill in velocity mode", context do
      %{state: state, servo_table: servo_table} = armed_state(context)
      state = %{state | mode: :velocity}

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Hold{}}, state)

      assert goal(servo_table, :goal_speed) == 0.0
    end

    test "resumes a passive servo where it came to rest", context do
      %{state: state} = passive_state(context)

      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:resume_servo, 1, [{:present_position, nil}]} ->
        :ok
      end)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Hold{}}, state)
    end
  end

  describe "Command.Velocity" do
    setup :armed_state

    test "writes the goal speed", %{state: state, servo_table: servo_table} do
      cmd = %Command.Velocity{velocity: 0.5}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert_in_delta goal(servo_table, :goal_speed), 0.5, 0.001
    end

    test "keeps the sign, because direction is the point", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Velocity{velocity: -0.5}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert_in_delta goal(servo_table, :goal_speed), -0.5, 0.001
    end

    test "clamps to the joint's velocity limit in both directions", %{
      state: state,
      servo_table: servo_table
    } do
      assert {:noreply, _state} =
               Actuator.handle_command(
                 %Message{payload: %Command.Velocity{velocity: 99.0}},
                 state
               )

      assert_in_delta goal(servo_table, :goal_speed), @pi / 3, 0.001

      assert {:noreply, _state} =
               Actuator.handle_command(
                 %Message{payload: %Command.Velocity{velocity: -99.0}},
                 state
               )

      assert_in_delta goal(servo_table, :goal_speed), -@pi / 3, 0.001
    end
  end

  describe "Command.Effort" do
    setup :armed_state

    test "writes a torque_limit fraction of the stall torque", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Effort{effort: 0.5}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      # 0.5 Nm against a 2.0 Nm stall torque is a quarter of what it can do.
      assert_in_delta torque_limit(servo_table), 0.25, 0.001
    end

    test "a ceiling has no direction", %{state: state, servo_table: servo_table} do
      cmd = %Command.Effort{effort: -0.5}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert_in_delta torque_limit(servo_table), 0.25, 0.001
    end

    test "asking for more than the servo has is asking for all of it", %{
      state: state,
      servo_table: servo_table
    } do
      cmd = %Command.Effort{effort: 99.0}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert_in_delta torque_limit(servo_table), 1.0, 0.001
    end

    test "clamps to the joint's effort limit when it has one", %{state: state} do
      state = %{state | motor_profile: motor_profile(motor_effort_limit: 1.0)}
      servo_table = state.servo_table

      cmd = %Command.Effort{effort: 99.0}

      assert {:noreply, _state} = Actuator.handle_command(%Message{payload: cmd}, state)
      # Capped at the joint's 1.0 Nm rather than the servo's 2.0 Nm.
      assert_in_delta torque_limit(servo_table), 0.5, 0.001
    end

    test "survives a motion command arriving in the same tick", %{
      state: state,
      servo_table: servo_table
    } do
      assert {:noreply, state} =
               Actuator.handle_command(%Message{payload: %Command.Effort{effort: 0.5}}, state)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Position{position: 0.5}}, state)

      # The move replaces the goal, but the ceiling is not a goal.
      assert_in_delta torque_limit(servo_table), 0.25, 0.001
      assert Keyword.has_key?(pending_writes(servo_table), :goal_position)
    end

    test "a ceiling alone doesn't move a passive joint, so a resume carries a standstill",
         context do
      %{state: state} = passive_state(context)

      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:resume_servo, 1, writes} ->
        assert writes == [{:torque_limit, 0.25}, {:present_position, nil}]
        :ok
      end)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Effort{effort: 0.5}}, state)
    end
  end

  describe "resuming a passive joint" do
    setup :passive_state

    test "a position command goes through the controller's ordered resume", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:resume_servo, 1, writes} ->
        assert Keyword.fetch!(writes, :goal_position) == 2048
        :ok
      end)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Position{position: 0.0}}, state)
    end

    test "the goal doesn't also land in the ETS table", %{
      state: state,
      servo_table: servo_table
    } do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:resume_servo, _, _} -> :ok end)

      assert {:noreply, _state} =
               Actuator.handle_command(%Message{payload: %Command.Position{position: 0.0}}, state)

      assert pending_writes(servo_table) == nil
    end

    test "a failed resume stops the actuator", %{state: state} do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:resume_servo, _, _} -> {:error, :timeout} end)

      assert {:stop, :timeout, _state} =
               Actuator.handle_command(%Message{payload: %Command.Position{position: 0.0}}, state)
    end
  end

  describe "command expiry" do
    setup :armed_state

    test "a velocity command with a duration schedules an expiry", %{state: state} do
      cmd = %Command.Velocity{velocity: 0.5, duration: 50}

      assert {:noreply, state} = Actuator.handle_command(%Message{payload: cmd}, state)
      assert is_reference(state.expiry_timer)

      assert_receive {:expire_command, timer} when timer == state.expiry_timer, 500
    end

    test "expiring under :stop cuts torque", %{state: state} do
      state = %{state | expiry_action: :stop, expiry_timer: timer = make_ref()}

      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :torque_enable, false} -> :ok end)

      assert {:noreply, state} = Actuator.handle_info({:expire_command, timer}, state)
      assert is_nil(state.expiry_timer)
    end

    test "expiring under :hold stays under power", %{state: state, servo_table: servo_table} do
      timer = make_ref()
      state = %{state | expiry_action: :hold, expiry_timer: timer, mode: :velocity}

      assert {:noreply, _state} = Actuator.handle_info({:expire_command, timer}, state)
      assert goal(servo_table, :goal_speed) == 0.0
    end

    test "expiring under :hold writes nothing in position mode", %{
      state: state,
      servo_table: servo_table
    } do
      timer = make_ref()
      state = %{state | expiry_action: :hold, expiry_timer: timer}

      assert {:noreply, _state} = Actuator.handle_info({:expire_command, timer}, state)

      # A powered servo already holds its goal. The `:present_position` sentinel
      # is only resolvable on the resume path — writing it here would send a
      # nil to a read-only register.
      assert pending_writes(servo_table) == nil
    end

    test "a stale expiry doesn't cut a newer command short", %{state: state} do
      state = %{state | expiry_timer: make_ref()}
      stale = make_ref()

      BB.Process
      |> reject(:call, 3)

      assert {:noreply, ^state} = Actuator.handle_info({:expire_command, stale}, state)
    end

    test "a new command cancels the previous expiry", %{state: state} do
      state = %{state | expiry_timer: make_ref()}

      assert {:noreply, state} =
               Actuator.handle_command(%Message{payload: %Command.Position{position: 0.0}}, state)

      assert is_nil(state.expiry_timer)
    end
  end

  # --- Test helpers ---

  defp base_opts(overrides \\ []) do
    Keyword.merge(
      [
        bb: %{robot: TestRobot, path: [:shoulder, :servo]},
        servo_id: 1,
        controller: :feetech,
        motor_profile: motor_profile()
      ],
      overrides
    )
  end

  defp drain_messages(acc \\ []) do
    receive do
      message -> drain_messages([message | acc])
    after
      0 -> Enum.reverse(acc)
    end
  end

  defp armed_state(context), do: powered_state(context, true)

  defp passive_state(context), do: powered_state(context, false)

  defp powered_state(_context, torque_enabled) do
    servo_table = :ets.new(:test_servo_table, [:set, :public])

    :ets.insert(
      servo_table,
      {1, [:shoulder, :servo], 2, nil, nil, nil, nil, nil, nil, nil, nil, torque_enabled}
    )

    state = %State{
      bb: %{robot: TestRobot, path: [:shoulder, :servo]},
      controller: :feetech,
      current_motor_angle: 0.0,
      joint_name: :shoulder,
      mode: :position,
      motor_profile: motor_profile(),
      name: :servo,
      servo_id: 1,
      servo_table: servo_table,
      stall_torque: 2.0
    }

    BB.Actuator
    |> stub(:publish_begin_motion, fn _robot, _path, _opts -> :ok end)

    on_exit(fn ->
      if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
    end)

    %{state: state, servo_table: servo_table}
  end

  defp pending_writes(servo_table) do
    [{1, _, _, _, _, _, _, _, _, pending_writes, _, _}] = :ets.lookup(servo_table, 1)
    pending_writes
  end

  defp goal(servo_table, param), do: Keyword.fetch!(pending_writes(servo_table), param)

  defp torque_limit(servo_table) do
    [{1, _, _, _, _, _, _, _, _, _, pending_limit, _}] = :ets.lookup(servo_table, 1)
    pending_limit
  end
end
