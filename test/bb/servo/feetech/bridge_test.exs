# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.BridgeTest do
  use ExUnit.Case, async: false
  use Mimic

  alias BB.Error.Invalid.Bridge.InvalidParamId
  alias BB.Error.Invalid.Bridge.ReadOnly
  alias BB.Error.Invalid.Bridge.TorqueMustBeDisabled
  alias BB.Error.Invalid.Bridge.UnknownParam
  alias BB.Servo.Feetech.Bridge

  @control_table Feetech.ControlTable.STS3215

  setup :verify_on_exit!

  describe "init/1" do
    test "queries controller for control table" do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, :get_control_table ->
        {:ok, @control_table}
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:feetech_bridge]},
        controller: :feetech
      ]

      assert {:ok, state} = Bridge.init(opts)
      assert state.robot == TestRobot
      assert state.controller == :feetech
      assert state.control_table == @control_table
    end
  end

  describe "list_remote/1" do
    setup do
      state = %{
        robot: TestRobot,
        controller: :feetech,
        control_table: @control_table
      }

      %{state: state}
    end

    test "returns parameters for all registered servos", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, :list_servos ->
        {:ok, [1, 2]}
      end)

      {:ok, params, ^state} = Bridge.list_remote(state)

      # Should have params for both servos
      servo_1_params = Enum.filter(params, &String.starts_with?(&1.id, "1:"))
      servo_2_params = Enum.filter(params, &String.starts_with?(&1.id, "2:"))

      assert servo_1_params != []
      assert servo_2_params != []
      assert length(servo_1_params) == length(servo_2_params)
    end

    test "includes parameter metadata", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, :list_servos ->
        {:ok, [1]}
      end)

      {:ok, params, _state} = Bridge.list_remote(state)

      # Find position_p_gain (a config param)
      gain_param = Enum.find(params, &(&1.id == "1:position_p_gain"))

      assert gain_param != nil
      assert gain_param.writable == true
      assert gain_param.category == :config
      assert gain_param.requires_torque_off == true
      assert is_binary(gain_param.doc)
    end

    test "returns empty list when no servos registered", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, :list_servos ->
        {:ok, []}
      end)

      {:ok, params, ^state} = Bridge.list_remote(state)
      assert params == []
    end
  end

  describe "get_remote/2" do
    setup do
      state = %{
        robot: TestRobot,
        controller: :feetech,
        control_table: @control_table
      }

      %{state: state}
    end

    test "reads parameter from controller", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:read, 1, :position_p_gain} ->
        {:ok, 50}
      end)

      assert {:ok, 50, ^state} = Bridge.get_remote("1:position_p_gain", state)
    end

    test "returns error for invalid param_id format", %{state: state} do
      assert {:error, %InvalidParamId{param_id: "invalid"}, ^state} =
               Bridge.get_remote("invalid", state)
    end

    test "returns error for non-numeric servo_id", %{state: state} do
      assert {:error, %InvalidParamId{param_id: "abc:position_p_gain"}, ^state} =
               Bridge.get_remote("abc:position_p_gain", state)
    end

    test "returns error for unknown parameter", %{state: state} do
      assert {:error, %UnknownParam{param_name: :nonexistent}, ^state} =
               Bridge.get_remote("1:nonexistent", state)
    end
  end

  describe "set_remote/3" do
    setup do
      state = %{
        robot: TestRobot,
        controller: :feetech,
        control_table: @control_table
      }

      %{state: state}
    end

    test "writes control parameter without torque check", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :goal_speed, 100} ->
        :ok
      end)

      assert {:ok, ^state} = Bridge.set_remote("1:goal_speed", 100, state)
    end

    test "checks torque before writing config parameter", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:read, 1, :torque_enable} ->
        {:ok, false}
      end)
      |> expect(:call, fn TestRobot, :feetech, {:write, 1, :position_p_gain, 100} ->
        :ok
      end)

      assert {:ok, ^state} = Bridge.set_remote("1:position_p_gain", 100, state)
    end

    test "returns error when writing config param with torque enabled", %{state: state} do
      BB.Process
      |> expect(:call, fn TestRobot, :feetech, {:read, 1, :torque_enable} ->
        {:ok, true}
      end)

      assert {:error, %TorqueMustBeDisabled{param_name: :position_p_gain, servo_id: 1}, ^state} =
               Bridge.set_remote("1:position_p_gain", 100, state)
    end

    test "returns error for read-only parameter", %{state: state} do
      assert {:error, %ReadOnly{param_name: :firmware_version_main}, ^state} =
               Bridge.set_remote("1:firmware_version_main", 1, state)
    end

    test "returns error for unknown parameter", %{state: state} do
      assert {:error, %UnknownParam{param_name: :nonexistent}, ^state} =
               Bridge.set_remote("1:nonexistent", 100, state)
    end

    test "returns error for invalid param_id format", %{state: state} do
      assert {:error, %InvalidParamId{param_id: "bad_format"}, ^state} =
               Bridge.set_remote("bad_format", 100, state)
    end
  end

  describe "handle_change/3" do
    test "returns ok without changes (inbound bridge)" do
      state = %{robot: TestRobot, controller: :feetech, control_table: @control_table}

      assert {:ok, ^state} = Bridge.handle_change(TestRobot, %{}, state)
    end
  end
end
