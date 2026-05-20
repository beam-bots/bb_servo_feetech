# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoFeetech.Upgrade do
    @shortdoc "Lifts `reverse?` and the implicit centering offset into joint transmissions"
    @moduledoc """
    #{@shortdoc}

    `BB.Servo.Feetech.Actuator` no longer takes a `reverse?` option, and no
    longer derives a `center_angle` from joint limits. Both pieces of
    behaviour have moved into the joint-level `transmission` block in BB.

    This upgrader rewrites every robot module in the project: for each
    actuator with `BB.Servo.Feetech.Actuator` as its driver, it removes
    `reverse?:` from the actuator's options and inserts a `transmission`
    block on the parent joint when `reverse?: true` was present and/or the
    joint's limits were asymmetric (so that `(lower + upper) / 2` is the
    angle the servo's hardware centre used to map to).
    """

    use Igniter.Mix.Task

    alias BB.Igniter.Transmission

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{}
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      Transmission.lift_reverse_question(
        igniter,
        BB.Servo.Feetech.Actuator,
        lift_offset?: true
      )
    end
  end
end
