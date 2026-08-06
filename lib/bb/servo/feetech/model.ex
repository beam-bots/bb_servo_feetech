# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech.Model do
  @moduledoc """
  What each Feetech servo can do, keyed on the `model_number` it reports.

  These servos have no torque goal — `torque_limit` is a ceiling expressed as a
  fraction of the model's rated stall torque, so turning a `Command.Effort` in
  newton metres into a register value needs that rating, and the rating is not
  readable from the bus.

  ## The numbers are approximate, and 777 is worse than that

  Model number does not identify a servo. An STS3215 reports 777 whether it is a
  7.4 V 1:345 (19.5 kgf·cm), a 7.4 V 1:191 (27.4 kgf·cm), or a 12 V 1:345
  (30 kgf·cm) — a 1.5× spread. The figure below is the 1:345 7.4 V variant, the
  one on an SO-ARM100/101, because it is the most common and because
  under-estimating stall torque errs towards a lower ceiling. If you have one of
  the others, set `:stall_torque` on the actuator; nothing here can work it out
  for you.

  Even for the right variant, stall torque is quoted at the recommended supply
  voltage and falls with it, and the servo's own overload protection winds
  `torque_limit` down under sustained load. Treat effort as a rough ceiling
  rather than a calibrated torque.

  ## Coverage

  Only the models below are known. An unrecognised servo still drives in
  position and velocity mode — neither needs anything from this table — but
  effort is refused unless `:stall_torque` says what the servo can do.

  `SCS0009` (1284) is deliberately absent: it is an SCSCL-family servo with a
  different control table, which the `feetech` library does not implement.
  """

  @type t :: %{
          name: String.t(),
          stall_torque: float()
        }

  # 1 kgf·cm = 0.0980665 N·m.
  @models %{
    777 => %{name: "STS3215", stall_torque: 1.912},
    2825 => %{name: "STS3250", stall_torque: 4.903},
    11_272 => %{name: "SM8512BL", stall_torque: 8.336}
  }

  @doc """
  Look up a servo by the `model_number` it reports.

  Returns `:error` for a model this table doesn't know about.
  """
  @spec fetch(integer()) :: {:ok, t()} | :error
  def fetch(model_number), do: Map.fetch(@models, model_number)
end
