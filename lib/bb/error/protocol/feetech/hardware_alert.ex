# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Feetech.HardwareAlert do
  @moduledoc """
  Feetech servo hardware alert error.

  Raised when a Feetech servo reports hardware error status flags.
  The `alerts` field contains a list of active alert atoms.

  ## Alert Types

  - `:voltage_error` - Input voltage out of range
  - `:sensor_error` - Magnetic encoder malfunction
  - `:temperature_error` - Motor temperature too high
  - `:current_error` - Over current detected
  - `:overload_error` - Overload protection triggered

  Note: Bit 4 in the status byte indicates torque state, not an error.
  """
  use BB.Error,
    class: :protocol,
    fields: [:servo_id, :alerts, :raw_value]

  @type alert ::
          :voltage_error | :sensor_error | :temperature_error | :current_error | :overload_error

  @type t :: %__MODULE__{
          servo_id: non_neg_integer(),
          alerts: [alert()],
          raw_value: non_neg_integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  @alert_bits [
    {0, :voltage_error},
    {1, :sensor_error},
    {2, :temperature_error},
    {3, :current_error},
    {5, :overload_error}
  ]

  @doc """
  Creates a HardwareAlert from raw hardware error bits.

  Bit 4 (torque enabled) is filtered out as it is not an error condition.
  """
  @spec from_bits(non_neg_integer(), non_neg_integer()) :: t()
  def from_bits(servo_id, bits) when is_integer(bits) and bits > 0 do
    alerts =
      @alert_bits
      |> Enum.filter(fn {bit, _name} -> Bitwise.band(bits, Bitwise.bsl(1, bit)) != 0 end)
      |> Enum.map(fn {_bit, name} -> name end)

    %__MODULE__{servo_id: servo_id, alerts: alerts, raw_value: bits}
  end

  def message(%{servo_id: servo_id, alerts: alerts}) do
    alert_str = Enum.join(alerts, ", ")
    "Servo #{servo_id} hardware alert: #{alert_str}"
  end
end
