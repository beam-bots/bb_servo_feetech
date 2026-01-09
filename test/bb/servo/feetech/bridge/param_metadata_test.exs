# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Bridge.ParamMetadataTest do
  use ExUnit.Case, async: true

  alias BB.Servo.Feetech.Bridge.ParamMetadata

  @control_table Feetech.ControlTable.STS3215

  describe "list_params/1" do
    test "returns a sorted list of parameter names" do
      params = ParamMetadata.list_params(@control_table)

      assert is_list(params)
      assert params != []
      assert params == Enum.sort(params)
    end

    test "includes info parameters" do
      params = ParamMetadata.list_params(@control_table)

      assert :firmware_version_main in params
      assert :firmware_version_sub in params
      assert :servo_version_main in params
      assert :servo_version_sub in params
    end

    test "includes config parameters" do
      params = ParamMetadata.list_params(@control_table)

      assert :id in params
      assert :baud_rate in params
      assert :min_angle_limit in params
      assert :max_angle_limit in params
      assert :position_p_gain in params
      assert :mode in params
    end

    test "includes control parameters" do
      params = ParamMetadata.list_params(@control_table)

      assert :torque_enable in params
      assert :goal_position in params
      assert :goal_speed in params
      assert :acceleration in params
      assert :lock in params
    end

    test "excludes status parameters" do
      params = ParamMetadata.list_params(@control_table)

      refute :present_position in params
      refute :present_speed in params
      refute :present_load in params
      refute :present_voltage in params
      refute :present_temperature in params
      refute :hardware_error_status in params
      refute :moving in params
    end
  end

  describe "param_info/2" do
    test "returns info category for firmware version parameters" do
      assert {:ok, info} = ParamMetadata.param_info(@control_table, :firmware_version_main)

      assert info.category == :info
      assert info.writable == false
      assert info.requires_torque_off == false
      assert is_binary(info.doc)
    end

    test "returns config category for EEPROM parameters" do
      assert {:ok, info} = ParamMetadata.param_info(@control_table, :position_p_gain)

      assert info.category == :config
      assert info.writable == true
      assert info.requires_torque_off == true
      assert is_binary(info.doc)
    end

    test "returns control category for SRAM parameters" do
      assert {:ok, info} = ParamMetadata.param_info(@control_table, :torque_enable)

      assert info.category == :control
      assert info.writable == true
      assert info.requires_torque_off == false
      assert is_binary(info.doc)
    end

    test "returns error for unknown parameters" do
      assert {:error, :unknown_param} =
               ParamMetadata.param_info(@control_table, :nonexistent_param)
    end
  end

  describe "writable?/2" do
    test "returns false for info parameters" do
      refute ParamMetadata.writable?(@control_table, :firmware_version_main)
      refute ParamMetadata.writable?(@control_table, :servo_version_main)
    end

    test "returns true for config parameters" do
      assert ParamMetadata.writable?(@control_table, :id)
      assert ParamMetadata.writable?(@control_table, :position_p_gain)
      assert ParamMetadata.writable?(@control_table, :mode)
    end

    test "returns true for control parameters" do
      assert ParamMetadata.writable?(@control_table, :torque_enable)
      assert ParamMetadata.writable?(@control_table, :goal_position)
    end

    test "returns false for unknown parameters" do
      refute ParamMetadata.writable?(@control_table, :nonexistent_param)
    end
  end

  describe "requires_torque_off?/2" do
    test "returns false for info parameters" do
      refute ParamMetadata.requires_torque_off?(@control_table, :firmware_version_main)
    end

    test "returns true for config parameters" do
      assert ParamMetadata.requires_torque_off?(@control_table, :id)
      assert ParamMetadata.requires_torque_off?(@control_table, :baud_rate)
      assert ParamMetadata.requires_torque_off?(@control_table, :position_p_gain)
      assert ParamMetadata.requires_torque_off?(@control_table, :mode)
    end

    test "returns false for control parameters" do
      refute ParamMetadata.requires_torque_off?(@control_table, :torque_enable)
      refute ParamMetadata.requires_torque_off?(@control_table, :goal_position)
      refute ParamMetadata.requires_torque_off?(@control_table, :goal_speed)
    end

    test "returns false for unknown parameters" do
      refute ParamMetadata.requires_torque_off?(@control_table, :nonexistent_param)
    end
  end

  describe "status_param?/1" do
    test "returns true for status parameters" do
      assert ParamMetadata.status_param?(:present_position)
      assert ParamMetadata.status_param?(:present_speed)
      assert ParamMetadata.status_param?(:present_load)
      assert ParamMetadata.status_param?(:present_voltage)
      assert ParamMetadata.status_param?(:present_temperature)
      assert ParamMetadata.status_param?(:hardware_error_status)
      assert ParamMetadata.status_param?(:moving)
      assert ParamMetadata.status_param?(:present_current)
    end

    test "returns false for non-status parameters" do
      refute ParamMetadata.status_param?(:torque_enable)
      refute ParamMetadata.status_param?(:goal_position)
      refute ParamMetadata.status_param?(:position_p_gain)
      refute ParamMetadata.status_param?(:firmware_version_main)
    end
  end
end
