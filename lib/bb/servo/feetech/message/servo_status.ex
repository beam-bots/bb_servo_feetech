# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Message.ServoStatus do
  @moduledoc """
  Status information for a Feetech servo.

  Published periodically by the controller when status polling is enabled.
  Subscribe to `[:sensor, :controller_name, :servo_status]` to receive updates.

  ## Fields

  - `servo_id` - The servo ID (1-253)
  - `temperature` - Internal temperature in Celsius
  - `voltage` - Input voltage in Volts
  - `load` - Present load as a percentage (-100 to 100)
  - `hardware_error` - Hardware error flags (nil if no errors)

  ## Hardware Error Flags

  The `hardware_error` field contains bit flags indicating error conditions:

  - Bit 0: Voltage error
  - Bit 1: Sensor error (magnetic encoder)
  - Bit 2: Temperature error
  - Bit 3: Current error
  - Bit 5: Overload error

  Note: Bit 4 indicates torque state, not an error.

  ## Examples

      alias BB.Servo.Feetech.Message.ServoStatus

      {:ok, msg} = ServoStatus.new(:feetech,
        servo_id: 1,
        temperature: 45.0,
        voltage: 7.2,
        load: 25.0,
        hardware_error: nil
      )
  """

  defstruct [
    :servo_id,
    :temperature,
    :voltage,
    :load,
    :hardware_error
  ]

  use BB.Message,
    schema: [
      servo_id: [type: :pos_integer, required: true, doc: "Servo ID (1-253)"],
      temperature: [type: :number, required: true, doc: "Temperature in Celsius"],
      voltage: [type: :number, required: true, doc: "Input voltage in Volts"],
      load: [type: :number, required: true, doc: "Load percentage (-100 to 100)"],
      hardware_error: [
        type: {:or, [:non_neg_integer, {:literal, nil}]},
        default: nil,
        doc: "Hardware error flags (nil if no errors)"
      ]
    ]

  @type t :: %__MODULE__{
          servo_id: pos_integer(),
          temperature: number(),
          voltage: number(),
          load: number(),
          hardware_error: non_neg_integer() | nil
        }
end
