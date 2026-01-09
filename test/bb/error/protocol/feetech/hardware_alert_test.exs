# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Feetech.HardwareAlertTest do
  use ExUnit.Case, async: true

  alias BB.Error.Protocol.Feetech.HardwareAlert
  alias BB.Error.Severity

  describe "from_bits/2" do
    test "parses voltage error (bit 0)" do
      alert = HardwareAlert.from_bits(1, 0x01)

      assert alert.servo_id == 1
      assert alert.raw_value == 0x01
      assert :voltage_error in alert.alerts
      assert length(alert.alerts) == 1
    end

    test "parses sensor error (bit 1)" do
      alert = HardwareAlert.from_bits(2, 0x02)

      assert alert.servo_id == 2
      assert alert.raw_value == 0x02
      assert :sensor_error in alert.alerts
      assert length(alert.alerts) == 1
    end

    test "parses temperature error (bit 2)" do
      alert = HardwareAlert.from_bits(3, 0x04)

      assert alert.servo_id == 3
      assert alert.raw_value == 0x04
      assert :temperature_error in alert.alerts
      assert length(alert.alerts) == 1
    end

    test "parses current error (bit 3)" do
      alert = HardwareAlert.from_bits(4, 0x08)

      assert alert.servo_id == 4
      assert alert.raw_value == 0x08
      assert :current_error in alert.alerts
      assert length(alert.alerts) == 1
    end

    test "parses overload error (bit 5)" do
      alert = HardwareAlert.from_bits(5, 0x20)

      assert alert.servo_id == 5
      assert alert.raw_value == 0x20
      assert :overload_error in alert.alerts
      assert length(alert.alerts) == 1
    end

    test "ignores bit 4 (torque enabled state)" do
      # Bit 4 (0x10) is torque_enabled, not an error
      alert = HardwareAlert.from_bits(1, 0x10)

      assert alert.alerts == []
      assert alert.raw_value == 0x10
    end

    test "parses multiple errors" do
      # Voltage (0x01) + temperature (0x04) + overload (0x20) = 0x25
      alert = HardwareAlert.from_bits(1, 0x25)

      assert alert.servo_id == 1
      assert alert.raw_value == 0x25
      assert :voltage_error in alert.alerts
      assert :temperature_error in alert.alerts
      assert :overload_error in alert.alerts
      assert length(alert.alerts) == 3
    end

    test "parses all error bits together" do
      # All error bits: 0x01 | 0x02 | 0x04 | 0x08 | 0x20 = 0x2F
      alert = HardwareAlert.from_bits(1, 0x2F)

      assert :voltage_error in alert.alerts
      assert :sensor_error in alert.alerts
      assert :temperature_error in alert.alerts
      assert :current_error in alert.alerts
      assert :overload_error in alert.alerts
      assert length(alert.alerts) == 5
    end

    test "filters out torque bit when mixed with errors" do
      # Voltage (0x01) + torque_enabled (0x10) + overload (0x20) = 0x31
      alert = HardwareAlert.from_bits(1, 0x31)

      assert :voltage_error in alert.alerts
      assert :overload_error in alert.alerts
      refute :torque_enabled in alert.alerts
      assert length(alert.alerts) == 2
    end
  end

  describe "message/1" do
    test "formats single error message" do
      alert = HardwareAlert.from_bits(1, 0x04)
      message = HardwareAlert.message(alert)

      assert message =~ "Servo 1"
      assert message =~ "temperature_error"
    end

    test "formats multiple errors message" do
      alert = HardwareAlert.from_bits(5, 0x25)
      message = HardwareAlert.message(alert)

      assert message =~ "Servo 5"
      assert message =~ "voltage_error"
      assert message =~ "temperature_error"
      assert message =~ "overload_error"
    end
  end

  describe "severity" do
    test "returns critical severity" do
      alert = HardwareAlert.from_bits(1, 0x01)
      assert Severity.severity(alert) == :critical
    end
  end
end
