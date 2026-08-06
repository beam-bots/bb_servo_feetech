# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Feetech.StallTorque do
  @moduledoc """
  A servo's rated stall torque isn't known, so effort can't be scaled.

  Raised at startup, once the servo on the bus has reported its model number.
  These servos take a torque ceiling as a fraction of what they can produce, so
  turning a `BB.Message.Actuator.Command.Effort` in newton metres into a
  `torque_limit` needs the rating — and nothing on the bus reports it.

  Every mode accepts `Effort`, so an unrecognised servo refuses to start rather
  than accepting effort commands it would have to guess at. Set `:stall_torque`
  on the actuator from the servo's specification table and it will start.
  """
  use BB.Error,
    class: :invalid,
    fields: [:servo_id, :model_number]

  @type t :: %__MODULE__{
          servo_id: non_neg_integer(),
          model_number: integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(error) do
    "Servo #{error.servo_id} reports an unrecognised model number " <>
      "(#{error.model_number}), so its rated stall torque is unknown and effort " <>
      "in newton metres can't be scaled. Set `stall_torque:` on the actuator. " <>
      "See BB.Servo.Feetech.Model."
  end
end
