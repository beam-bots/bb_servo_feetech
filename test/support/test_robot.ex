# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule TestRobot do
  @moduledoc """
  Mock robot module for testing.

  Provides a minimal robot struct for actuator and controller tests.
  """

  @pi :math.pi()

  def robot do
    %BB.Robot{
      name: :test_robot,
      root_link: nil,
      joints: %{
        shoulder: %{
          type: :revolute,
          limits: %{
            lower: -@pi / 2,
            upper: @pi / 2,
            velocity: @pi / 3,
            effort: nil
          }
        },
        elbow: %{
          type: :revolute,
          limits: %{
            lower: 0,
            upper: @pi,
            velocity: @pi / 2,
            effort: nil
          }
        },
        wheel: %{
          type: :continuous,
          limits: nil
        },
        joint: %{
          type: :revolute,
          limits: nil
        },
        missing: nil
      },
      links: %{},
      sensors: %{},
      actuators: %{},
      topology: [],
      param_subscriptions: %{}
    }
  end
end
