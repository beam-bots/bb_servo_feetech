# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbServoFeetech.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "controller" do
    test "uses param refs for port and baud_rate" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :feetech,
      + |      {BB.Servo.Feetech.Controller,
      + |       port: param([:config, :feetech, :device]),
      + |       baud_rate: param([:config, :feetech, :baud_rate]),
      + |       control_table: Feetech.ControlTable.STS3215}
      + |    )
      """)
    end

    test "uses a custom controller name when --name is given" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install", ["--name", "sts"])
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :sts,
      """)
    end
  end

  describe "bridge" do
    test "adds a parameter bridge wired to the controller" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    bridge(:feetech_bridge, {BB.Servo.Feetech.Bridge, controller: :feetech})
      """)
    end
  end

  describe "parameters group" do
    test "adds a :config.:feetech param group with device" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    group :config do
      + |      group :feetech do
      + |        param(:device, type: :string, doc: "Serial device connected to the Feetech servo bus")
      """)
    end
  end

  describe "application module" do
    test "sets the device path on the robot child spec" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [feetech: [device: "/dev/ttyUSB0"]]]]}]
      ''')
    end

    test "honours a custom --device option" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install", ["--device", "/dev/ttyACM0"])
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [feetech: [device: "/dev/ttyACM0"]]]]}]
      ''')
    end
  end

  describe "formatter" do
    test "imports bb_servo_feetech into .formatter.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_patch(".formatter.exs", """
      + |  import_deps: [:bb_servo_feetech, :bb]
      """)
    end
  end

  describe "notice" do
    test "prints a topology snippet for the user to paste" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_notice(&String.contains?(&1, "BB.Servo.Feetech.Actuator"))
    end
  end

  describe "idempotency" do
    test "running twice produces no further changes" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_unchanged()
    end

    test "skips when a Feetech controller is already present under a different name" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install", ["--name", "feetech_controller"])
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_unchanged()
    end

    test "prints a notice when skipping a duplicate install" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_feetech.install", ["--name", "feetech_controller"])
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_feetech.install")
      |> assert_has_notice(&String.contains?(&1, "already has a BB.Servo.Feetech.Controller"))
    end
  end
end
