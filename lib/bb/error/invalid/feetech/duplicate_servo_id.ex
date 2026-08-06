# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Feetech.DuplicateServoId do
  @moduledoc """
  Two actuators on one controller claim the same servo ID.

  Raised when the second of them registers. A servo ID addresses one servo on
  one bus, and the controller keys its shared table on it, so a second claim
  would take over the first actuator's row: only the later joint would receive
  position feedback, and the two would overwrite each other's pending commands
  every tick.

  This is a configuration mistake rather than a wiring one. Two *servos* sharing
  an ID is a different problem, and one the bus reports for itself — they both
  answer the same read and the frames collide. Use `mix feetech.scan` to see
  what is actually out there.
  """
  use BB.Error,
    class: :invalid,
    fields: [:servo_id, :actuator_path, :registered_path]

  @type t :: %__MODULE__{
          servo_id: non_neg_integer(),
          actuator_path: [atom()],
          registered_path: [atom()]
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(error) do
    "Servo #{error.servo_id} is already driven by " <>
      "#{inspect(error.registered_path)}, so #{inspect(error.actuator_path)} " <>
      "cannot drive it too. Each actuator needs its own `servo_id`."
  end
end
