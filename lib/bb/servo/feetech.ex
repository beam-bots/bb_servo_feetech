# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Feetech do
  @moduledoc """
  Beam Bots integration for Feetech/WaveShare serial bus servos.

  Provides controller, actuator, and bridge modules for STS/SCS series servos
  (STS3215, STS3032, etc.) using the `feetech` library for serial communication.

  ## Features

  - Closed-loop position feedback via controller polling
  - Multiple servos on a single bus (IDs 1-253)
  - Safety integration with automatic torque disable
  - Parameter bridge for servo configuration
  - Status monitoring (temperature, voltage, load, errors)

  ## Example Usage

      defmodule MyRobot do
        use BB

        controller :feetech, {BB.Servo.Feetech.Controller,
          port: "/dev/ttyUSB0",
          baud_rate: 1_000_000,
          control_table: Feetech.ControlTable.STS3215
        }

        parameters do
          bridge :feetech, {BB.Servo.Feetech.Bridge, controller: :feetech}
        end

        topology do
          link :base do
            joint :shoulder, type: :revolute do
              limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

              actuator :servo, {BB.Servo.Feetech.Actuator,
                servo_id: 1,
                controller: :feetech
              }
            end
          end
        end
      end

  ## Architecture

  The package follows a controller/actuator pattern:

  - **Controller** - Single GenServer managing the Feetech serial bus, position
    polling, status monitoring, and torque control for all registered servos.

  - **Actuator** - GenServer for each servo that receives position commands,
    converts radians to servo units, and publishes motion messages.

  - **Bridge** - Optional parameter bridge exposing servo control table parameters
    through the BB parameter system.

  ## Modules

  - `BB.Servo.Feetech.Controller` - Serial bus management
  - `BB.Servo.Feetech.Actuator` - Individual servo control
  - `BB.Servo.Feetech.Bridge` - Parameter bridge
  - `BB.Servo.Feetech.Message.ServoStatus` - Status message struct
  - `BB.Error.Protocol.Feetech.HardwareAlert` - Hardware error representation
  """
end
