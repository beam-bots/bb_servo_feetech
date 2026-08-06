# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Feetech.UnknownController do
  @moduledoc """
  An actuator's `:controller` doesn't name a controller the robot declares.

  Raised at startup, before the actuator tries to talk to the bus. Without the
  check the first call would exit with a bare `:noproc` from somewhere inside
  `init/1`, the actuator would restart until the topology supervisor gave up,
  and nothing along the way would mention the name that was wrong.

  Checked against the DSL rather than the registry, so the message can list the
  controllers that do exist. Controllers all start before any actuator, so a
  name that isn't there is a typo rather than a race.
  """
  use BB.Error,
    class: :invalid,
    fields: [:controller, :actuator_path, :known]

  @type t :: %__MODULE__{
          controller: atom(),
          actuator_path: [atom()],
          known: [atom()]
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{known: []} = error) do
    "#{inspect(error.actuator_path)} names controller " <>
      "#{inspect(error.controller)}, but the robot declares no controllers at all."
  end

  def message(error) do
    "#{inspect(error.actuator_path)} names controller " <>
      "#{inspect(error.controller)}, which the robot doesn't declare. " <>
      "It has: #{Enum.map_join(error.known, ", ", &inspect/1)}."
  end
end
