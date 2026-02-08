<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots Feetech/Waveshare servo control

[![CI](https://github.com/beam-bots/bb_servo_feetech/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_servo_feetech/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_servo_feetech.svg)](https://hex.pm/packages/bb_servo_feetech)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_servo_feetech)](https://api.reuse.software/info/github.com/beam-bots/bb_servo_feetech)

# BB.Servo.Feetech

BB integration for driving Feetech/WaveShare STS/SCS series serial bus servos.

This library provides controller, actuator, and parameter bridge modules for
integrating Feetech servos with the Beam Bots robotics framework. Unlike
PWM servos, Feetech serial bus servos provide closed-loop position feedback.

## Features

- **Closed-loop position feedback** - Servos report their actual position
- **Multiple servos on one bus** - All servos share a single serial connection
- **Safety integration** - Torque is automatically disabled when the robot is disarmed or crashes
- **Status monitoring** - Temperature, voltage, load, and hardware error reporting
- **Parameter access** - Read and write servo configuration via the BB parameter system

## Supported Servos

Currently supports the following servo families via USB-to-TTL adapters:

- STS3215
- STS3032
- SCS series (using appropriate control table)

## Installation

Add `bb_servo_feetech` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_servo_feetech, "~> 0.2.1"}
  ]
end
```

## Requirements

- USB-to-TTL serial adapter compatible with Feetech servos
- Feetech STS/SCS series servos
- BB framework (`~> 0.12`)

## Usage

Define a controller and joints with servo actuators in your robot DSL:

```elixir
defmodule MyRobot do
  use BB

  controller :feetech, {BB.Servo.Feetech.Controller,
    port: "/dev/ttyUSB0",
    baud_rate: 1_000_000,
    control_table: Feetech.ControlTable.STS3215
  }

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Feetech.Actuator,
          servo_id: 1,
          controller: :feetech
        }

        link :upper_arm do
          joint :elbow, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.Feetech.Actuator,
              servo_id: 2,
              controller: :feetech
            }

            link :forearm do
            end
          end
        end
      end
    end
  end
end
```

The actuator automatically derives its configuration from the joint limits - no
need to specify servo rotation range or speed separately. Position feedback is
handled by the controller; no separate sensor is needed.

## Sending Commands

Use the `BB.Actuator` module to send commands to servos. Three delivery methods
are available:

### Pubsub Delivery (for orchestration)

Commands are published via pubsub, enabling logging, replay, and multi-subscriber
patterns:

```elixir
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5,
  command_id: make_ref()
)
```

### Direct Delivery (for time-critical control)

Commands bypass pubsub for lower latency:

```elixir
BB.Actuator.set_position!(MyRobot, :servo, 0.5)
```

### Synchronous Delivery (with acknowledgement)

Wait for the actuator to acknowledge the command:

```elixir
case BB.Actuator.set_position_sync(MyRobot, :servo, 0.5) do
  {:ok, :accepted} -> :ok
  {:error, reason} -> handle_error(reason)
end
```

## Components

### Controller

`BB.Servo.Feetech.Controller` manages the serial connection to the Feetech
bus. Define one controller per serial adapter. The controller handles:

- Serial communication with servos
- Position feedback polling for all registered servos
- Status monitoring (temperature, voltage, load, errors)
- Torque enable/disable on arm/disarm

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `port` | string | required | Serial port path (e.g., "/dev/ttyUSB0") |
| `baud_rate` | integer | 1000000 | Baud rate in bps |
| `control_table` | module | `Feetech.ControlTable.STS3215` | Servo control table |
| `poll_interval_ms` | integer | 50 | Position feedback polling interval (20Hz default) |
| `status_poll_interval_ms` | integer | 1000 | Status polling interval (0 to disable) |
| `disarm_action` | atom | `:disable_torque` | Action on disarm (`:disable_torque` or `:hold`) |

### Actuator

`BB.Servo.Feetech.Actuator` controls a single servo on the bus.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `servo_id` | 1-253 | required | Feetech servo ID |
| `controller` | atom | required | Name of the controller in robot registry |
| `reverse?` | boolean | false | Reverse rotation direction |
| `position_deadband` | integer | 2 | Minimum position change (raw units) to publish feedback |

**Behaviour:**

- Maps joint position limits to servo position (0-4095 units for 360 degrees)
- Clamps commanded positions to joint limits
- Registers with controller for position feedback
- Publishes `BB.Message.Actuator.BeginMotion` after each command
- Torque management is delegated to the controller

### Position Feedback

Unlike PWM servos, Feetech servos report their actual position. The controller
polls all registered servos and publishes `BB.Message.Sensor.JointState` messages.
No separate sensor is needed in the robot definition.

Subscribe to position updates:

```elixir
BB.subscribe(MyRobot, [:sensor, :feetech, :shoulder])
```

### Status Monitoring

The controller periodically reads status registers and publishes
`BB.Servo.Feetech.Message.ServoStatus` messages containing:

- `temperature` - Internal temperature in Celsius
- `voltage` - Input voltage in Volts
- `load` - Load percentage (0-100)
- `hardware_error` - Hardware error flags (nil if no errors)

Hardware errors are also reported to the safety system.

Subscribe to status updates:

```elixir
BB.subscribe(MyRobot, [:sensor, :feetech, :servo_status])
```

### Parameter Bridge

`BB.Servo.Feetech.Bridge` exposes servo configuration through the BB parameter
system. Parameters are identified by strings in the format `"servo_id:param_name"`.

```elixir
defmodule MyRobot do
  use BB

  parameters do
    bridge :feetech, {BB.Servo.Feetech.Bridge, controller: :feetech}
  end
end

# Read parameter
{:ok, 50} = BB.Parameter.get_remote(MyRobot, :feetech, "1:position_p_gain")

# Write parameter (control params)
:ok = BB.Parameter.set_remote(MyRobot, :feetech, "1:position_p_gain", 100)

# List all parameters
{:ok, params} = BB.Parameter.list_remote(MyRobot, :feetech)
```

Parameter categories:

- **info** - Read-only identification (firmware_version, servo_version)
- **config** - EEPROM settings, require torque off to write (limits, gains, mode)
- **control** - SRAM settings, writable at runtime (goal_position, torque_enable)

## How It Works

### Architecture

```
Controller (GenServer)
    |
    v wraps
Feetech (Serial communication)
    ^
    | used by
Actuator (GenServer) --publishes--> BeginMotion
    |
    v registers with
Controller --publishes--> JointState (position feedback)
          --publishes--> ServoStatus (status monitoring)
```

Multiple actuators share a single controller. Each actuator controls a servo
with a unique ID (1-253) on the bus.

### Position Mapping

The actuator maps the joint's position limits to the servo's position range
(4096 steps per revolution, centre at 2048):

```
Joint centre       ->  2048 (servo centre)
Joint lower limit  ->  2048 - (range/2) steps
Joint upper limit  ->  2048 + (range/2) steps
```

For a joint with limits `-90 degrees` to `+90 degrees`:
- `-90 degrees` maps to 1024 (90 degrees CCW from centre)
- `0 degrees` maps to 2048 (servo centre)
- `+90 degrees` maps to 3072 (90 degrees CW from centre)

### Safety

The controller implements `BB.Safety`:

- **On arm** - Torque is enabled on all registered servos
- **On disarm** - Torque is disabled (or held, if configured)
- **On crash** - Torque is disabled without requiring GenServer state

Hardware errors detected during status polling are reported to the safety
system and may trigger disarm depending on your safety configuration.

### Motion Lifecycle

When a position command is processed:

1. Actuator clamps position to joint limits
2. Converts angle to servo position (0-4095 units)
3. Sends goal_position to controller
4. Controller writes to servo via Feetech library
5. Publishes `BB.Message.Actuator.BeginMotion` with:
   - `initial_position` - where the servo was
   - `target_position` - where it's going
   - `expected_arrival` - when it should arrive (based on velocity limit)
   - `command_id` - correlation ID (if provided)
   - `command_type` - `:position`

Position feedback is handled separately by the controller's polling loop.

## Differences from bb_servo_robotis

While this package follows the same architecture as `bb_servo_robotis`, there
are some differences in the underlying servo protocol:

| Feature | Feetech | Robotis |
|---------|---------|---------|
| Position resolution | 4096 steps | 4096 steps |
| Load reporting | `present_load` (percentage) | `present_current` (mA) |
| Status byte bit 4 | Torque enabled (not an error) | Hardware error |
| Sync read | Returns list of values | Callback-based |

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_servo_feetech).
