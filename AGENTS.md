<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.Servo.Feetech is a Beam Bots integration library for driving Feetech/WaveShare serial
bus servos (STS/SCS series). It provides controller, actuator, and parameter bridge
modules that plug into the BB robotics framework's DSL.

Like Dynamixel servos, Feetech servos provide closed-loop position feedback, so no
separate sensor module is required.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Run single test at line
mix format              # Format code
mix credo --strict      # Linting
```

The project uses `ex_check` - always prefer `mix check --no-retry` over running individual tools.

## Architecture

### Component Hierarchy

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

Bridge (GenServer) --reads/writes--> Controller --reads/writes--> Servo registers
```

### Key Modules

- **Controller** (`lib/bb/servo/feetech/controller.ex`) - GenServer wrapping the `Feetech` library.
  Handles serial communication, position feedback polling (via `sync_read`), status monitoring,
  and torque management. Multiple actuators share one controller. Implements `BB.Controller` and
  `BB.Safety` behaviours.

- **Actuator** (`lib/bb/servo/feetech/actuator.ex`) - GenServer that receives position commands
  (radians), converts to servo position (0-4095), sends to controller, and publishes
  `BB.Message.Actuator.BeginMotion` messages. Handles commands via three delivery methods:
  - `handle_info/2` for pubsub delivery (`BB.Actuator.set_position/4`)
  - `handle_cast/2` for direct delivery (`BB.Actuator.set_position!/4`)
  - `handle_call/3` for synchronous delivery (`BB.Actuator.set_position_sync/5`)

- **Bridge** (`lib/bb/servo/feetech/bridge.ex`) - Parameter bridge exposing servo control table
  parameters through the BB parameter system. Parameters are identified as `"servo_id:param_name"`.

- **ParamMetadata** (`lib/bb/servo/feetech/bridge/param_metadata.ex`) - Metadata for control table
  parameters. Categorises parameters as info (read-only), config (requires torque off), or control
  (runtime writable).

- **ServoStatus** (`lib/bb/servo/feetech/message/servo_status.ex`) - Message struct for servo status
  information (temperature, voltage, load, hardware errors).

### BB Framework Integration

The library uses BB's:
- `BB.Controller` behaviour for controller lifecycle
- `BB.Actuator` behaviour for actuator lifecycle
- `BB.Bridge` behaviour for parameter bridge
- `BB.Message` for typed message payloads
- `BB.Safety` for arm/disarm handling
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call`/`BB.Process.cast` to communicate with sibling processes via the robot registry
- `Spark.Options` for configuration validation
- Joint limits from robot topology to derive servo parameters

### Command Interface

Send commands using the `BB.Actuator` module:

```elixir
# Pubsub delivery (for orchestration/logging)
BB.Actuator.set_position(MyRobot, [:joint, :servo], 0.5)

# Direct delivery (fire-and-forget, lower latency)
BB.Actuator.set_position!(MyRobot, :servo, 0.5)

# Synchronous delivery (with acknowledgement)
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :servo, 0.5)
```

### Integration Pattern

```elixir
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
```

### Testing

Tests use Mimic to mock `BB`, `BB.Process`, `BB.Robot`, and `Feetech`. Test support modules are
in `test/support/`.

## Dependencies

- `bb` - The Beam Bots robotics framework
- `feetech` - Feetech serial protocol driver

### Message Flow

```
BB.Actuator.set_position()
    |
    v
Actuator receives Command.Position
    |
    v
Actuator casts to Controller with goal_position
    |
    v
Controller writes to servo via Feetech library
    |
    v
Actuator publishes BeginMotion

Controller poll loop (separate):
    |
    v
Controller reads present_position via sync_read
    |
    v
Controller publishes JointState per joint

Controller status poll loop (separate):
    |
    v
Controller reads temperature/voltage/load/errors
    |
    v
Controller publishes ServoStatus
    |
    v
Controller reports hardware errors to BB.Safety
```

### Supported Control Tables

- `Feetech.ControlTable.STS3215` - STS3215 and compatible STS/SCS series servos

### Key Differences from BB.Servo.Robotis

| Aspect | Robotis | Feetech |
|--------|---------|---------|
| Sync read | `Robotis.fast_sync_read/3` | `Feetech.sync_read/3` |
| Return format | Values via callback | `{:ok, [values]}` list |
| Position units | Degrees (needs conversion) | Radians (already converted) |
| Current sensing | `present_current` | `present_load` (percentage) |
| Error bit 4 | Error flag | Torque enabled state |
| Default voltage | 10-14V | 5.5-8V |
