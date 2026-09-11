# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.Feetech.ControllerUnavailable do
  @moduledoc """
  An actuator's controller is declared by the robot but isn't running.

  Raised when a controller restarts. The actuator stops with it — every servo
  setting it made was written over a bus the replacement reopens — and races
  the controller on the way back up, because both supervisors react at once
  and the controller has a serial port to open first.

  The actuator loses that race, so it waits `:controller_grace` before
  reporting this, and its supervisor's restart budget decides how many times
  it will wait. Without the pause the restart is retried in microseconds:
  `Supervisor` re-runs a failed start with no backoff at all, so the budget
  would be spent before the port was open.

  Distinct from `BB.Error.Invalid.Feetech.UnknownController`, which means the
  name was never declared. This one means the name is right and the process
  isn't there yet.
  """
  use BB.Error,
    class: :hardware,
    fields: [:controller, :actuator_path, :waited_ms]

  @type t :: %__MODULE__{
          controller: atom(),
          actuator_path: [atom()],
          waited_ms: non_neg_integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(error) do
    "#{inspect(error.actuator_path)} found no controller registered as " <>
      "#{inspect(error.controller)} after waiting #{error.waited_ms}ms. " <>
      "Restarting to try again."
  end
end
