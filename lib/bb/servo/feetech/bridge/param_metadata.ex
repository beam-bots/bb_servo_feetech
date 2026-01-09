# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Bridge.ParamMetadata do
  @moduledoc """
  Metadata for Feetech servo control table parameters.

  Categorises parameters and determines access permissions for the parameter bridge.
  Status parameters (temperature, voltage, load, etc.) are excluded as they are
  exposed via sensor messages instead.

  ## Categories

  - `:info` - Read-only identification (firmware_version, servo_version)
  - `:config` - EEPROM settings, require torque off to write (limits, gains, mode)
  - `:control` - SRAM settings, writable at runtime (torque_enable, goal_position)
  """

  @type category :: :info | :config | :control
  @type control_table :: module

  @type param_info :: %{
          category: category(),
          writable: boolean(),
          requires_torque_off: boolean(),
          doc: String.t()
        }

  @info_params %{
    firmware_version_main: "Firmware major version",
    firmware_version_sub: "Firmware minor version",
    servo_version_main: "Servo hardware major version",
    servo_version_sub: "Servo hardware minor version"
  }

  @config_params %{
    id: "Servo ID (1-253)",
    baud_rate: "Communication baud rate",
    return_delay: "Response delay time",
    status_return_level: "Status packet return level",
    min_angle_limit: "Minimum angle limit",
    max_angle_limit: "Maximum angle limit",
    max_temperature: "Maximum temperature limit",
    max_input_voltage: "Maximum input voltage limit",
    min_input_voltage: "Minimum input voltage limit",
    max_torque: "Maximum torque limit",
    setting_byte: "Settings configuration byte",
    protection_switch: "Protection switch flags",
    led_alarm_condition: "LED alarm trigger conditions",
    position_p_gain: "Position PID P gain",
    position_d_gain: "Position PID D gain",
    position_i_gain: "Position PID I gain",
    punch: "Minimum PWM threshold",
    cw_dead_band: "Clockwise dead band",
    ccw_dead_band: "Counter-clockwise dead band",
    overload_current: "Overload current threshold",
    angular_resolution: "Angular resolution setting",
    position_offset: "Position offset from home",
    mode: "Operating mode (position/velocity/step)",
    protection_torque: "Protection torque threshold",
    protection_time: "Protection time duration",
    overload_torque: "Overload torque threshold"
  }

  @control_params %{
    torque_enable: "Enable/disable torque",
    acceleration: "Movement acceleration",
    goal_position: "Goal position",
    goal_time: "Time to reach goal",
    goal_speed: "Goal speed",
    torque_limit: "Torque limit",
    lock: "Lock EEPROM writes"
  }

  @status_params [
    :present_position,
    :present_speed,
    :present_load,
    :present_voltage,
    :present_temperature,
    :async_write_status,
    :hardware_error_status,
    :moving,
    :present_current
  ]

  @doc """
  List all parameter names for a control table, excluding status parameters.
  """
  @spec list_params(control_table()) :: [atom()]
  def list_params(_control_table) do
    all_params = Map.keys(@info_params) ++ Map.keys(@config_params) ++ Map.keys(@control_params)
    Enum.sort(all_params)
  end

  @doc """
  Get metadata for a specific parameter.
  """
  @spec param_info(control_table(), atom()) :: {:ok, param_info()} | {:error, :unknown_param}
  def param_info(_control_table, param_name) do
    cond do
      Map.has_key?(@info_params, param_name) ->
        {:ok,
         %{
           category: :info,
           writable: false,
           requires_torque_off: false,
           doc: @info_params[param_name]
         }}

      Map.has_key?(@config_params, param_name) ->
        {:ok,
         %{
           category: :config,
           writable: true,
           requires_torque_off: true,
           doc: @config_params[param_name]
         }}

      Map.has_key?(@control_params, param_name) ->
        {:ok,
         %{
           category: :control,
           writable: true,
           requires_torque_off: false,
           doc: @control_params[param_name]
         }}

      true ->
        {:error, :unknown_param}
    end
  end

  @doc """
  Check if a parameter is writable.
  """
  @spec writable?(control_table(), atom()) :: boolean()
  def writable?(control_table, param_name) do
    case param_info(control_table, param_name) do
      {:ok, %{writable: writable}} -> writable
      {:error, _} -> false
    end
  end

  @doc """
  Check if a parameter requires torque to be disabled before writing.
  """
  @spec requires_torque_off?(control_table(), atom()) :: boolean()
  def requires_torque_off?(control_table, param_name) do
    case param_info(control_table, param_name) do
      {:ok, %{requires_torque_off: requires}} -> requires
      {:error, _} -> false
    end
  end

  @doc """
  Check if a parameter is a status parameter (excluded from bridge).
  """
  @spec status_param?(atom()) :: boolean()
  def status_param?(param_name) do
    param_name in @status_params
  end
end
