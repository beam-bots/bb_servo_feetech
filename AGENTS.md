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
Actuator (GenServer)
    | writes goal_position/goal_speed
    v
Controller's ETS command table
    ^ drained by the controller's command loop
    |
Controller (GenServer) --wraps--> Feetech (serial comms) --> Servo registers

Actuator   --publishes--> BeginMotion
Controller --publishes--> JointState (position feedback)
           --publishes--> ServoStatus (status monitoring)

Bridge (GenServer) --reads/writes--> Controller --reads/writes--> Servo registers
```

### Key Modules

- **Controller** (`lib/bb/servo/feetech/controller.ex`) - GenServer wrapping the `Feetech` library.
  Handles serial communication, position feedback polling (via `sync_read`), status monitoring,
  and torque management. Multiple actuators share one controller. Implements the `BB.Controller`
  behaviour, including its `disarm/1` safety callback.

  Every write to `torque_enable` goes through the controller, so it caches each servo's torque
  state in the ETS row. That lets an actuator tell whether its goal will be acted on without a
  bus round trip. `{:resume_servo, id, writes}` is the ordered way back under power: the goals
  are written and acknowledged *before* torque comes on, so a servo that has drifted while
  passive doesn't lunge for the goal it was chasing when torque was cut.

  Pending writes are `{param, value}` pairs grouped by param each tick, so one `sync_write`
  covers every servo that wanted that register. Values are in the units `Feetech.write/5` takes
  — except `goal_position`, which is raw encoder units because this driver puts motor zero at
  the encoder midpoint rather than at the control table's signed zero.

- **Actuator** (`lib/bb/servo/feetech/actuator.ex`) - GenServer that receives position commands
  (radians), converts to servo position (0-4095), writes `goal_position`/`goal_speed` to the
  controller's ETS command table, and publishes `BB.Message.Actuator.BeginMotion` messages. Accepts commands sent via:
  - `BB.Actuator.set_position/4` (pubsub)
  - `BB.Actuator.set_position!/4` (direct)
  - `BB.Actuator.set_position_sync/5` (synchronous)

  All three arrive at `handle_command/2`; `BB.Actuator.Server` checks arm state and applies
  the joint's transmission before the driver sees them.

  It also handles `Command.Stop` (cut torque, joint goes passive) and `Command.Hold` (re-apply
  torque where the joint is now resting; a no-op if it never went passive). Any command to a
  joint left passive by a `Stop` resumes on the way past, so callers needn't pair the two.

  `:mode` fixes the servo's operating mode at startup and decides what `command_payloads/1`
  declares — `:position` or `:velocity`. Anything outside the mode's list is refused by the
  framework with `BB.Error.State.UnsupportedCommand`. The mode register is EEPROM, so it is
  written once with torque already off and the `lock` register cleared, and only when the servo
  isn't already in the right mode. The servo's `:pwm` and `:step` modes aren't offered.

  The joint's `motor_acceleration_limit` is written to the servo's `acceleration`
  register once at startup, and again if a parameter change alters the profile. A joint
  declaring no acceleration limit gets `0`, the servo's "no limit" — which is what
  `BB.Sim.Actuator` assumes when it falls back to a rectangular velocity profile.

  `Command.Effort` is a **ceiling, not a goal** — these servos have no torque goal register.
  It writes `torque_limit` as a fraction of the model's rated stall torque (see **Model**), so
  setting one won't move anything on its own.

- **Model** (`lib/bb/servo/feetech/model.ex`) - Rated stall torque per model number, used to
  scale `Command.Effort` into a `torque_limit` fraction. Rougher than the Robotis equivalent:
  an STS3215 reports 777 whether it is a 19.5, 27.4 or 30 kgf·cm variant, so the table carries
  the most common one and `:stall_torque` on the actuator overrides it. An unrecognised model
  refuses to start unless `:stall_torque` is set.

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
- `BB.Safety` API to register the controller and report hardware errors
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call`/`BB.Process.cast` to communicate with sibling processes via the robot registry
- `Spark.Options` for configuration validation
- Joint limits from robot topology to derive servo parameters

### Command Interface

Send commands using the `BB.Actuator` module:

```elixir
# Arm first — a disarmed robot refuses commands before they reach the driver
{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Pubsub delivery (for orchestration/logging). Takes a name or a full path.
BB.Actuator.set_position(MyRobot, :servo, 0.5)
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

# Direct delivery (fire-and-forget, lower latency)
BB.Actuator.set_position!(MyRobot, :servo, 0.5)

# Synchronous delivery (with acknowledgement)
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :servo, 0.5)

# Go passive — the joint can be backdriven by hand, and will sag under load
BB.Actuator.stop(MyRobot, :servo)

# Back under power, holding wherever it came to rest
BB.Actuator.hold(MyRobot, :servo)

# Only in an actuator configured `mode: :velocity`
BB.Actuator.set_velocity(MyRobot, :wheel, 2.0, duration: 500)

# A ceiling on what a move may draw, not a goal — moves nothing on its own
BB.Actuator.set_effort(MyRobot, :gripper, 0.4)
```

`stop/3` is not the safety path: it leaves the robot armed and commandable. Making the
hardware safe is `MyRobot.disarm()`, which is robot-wide.

### Integration Pattern

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :feetech, {BB.Servo.Feetech.Controller,
      port: "/dev/ttyUSB0",
      baud_rate: 1_000_000,
      control_table: Feetech.ControlTable.STS3215
    }
  end

  parameters do
    # Names are unique across the whole robot, so the bridge can't also be
    # called `:feetech` — that's already the controller.
    bridge :feetech_params, {BB.Servo.Feetech.Bridge, controller: :feetech}
  end

  topology do
    link :base do
      joint :shoulder do
        type :revolute

        limit lower: ~u(-90 degree), upper: ~u(90 degree),
              velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

        actuator :servo, {BB.Servo.Feetech.Actuator,
          servo_id: 1,
          controller: :feetech
        }

        link :upper_arm do
        end
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
Actuator writes goal_position/goal_speed to the controller's ETS command table
    |
    v
Actuator publishes BeginMotion

Controller command loop (separate):
    |
    v
Controller drains the ETS table and batches pending commands
    |
    v
Controller writes goal_position/goal_speed via Feetech sync_write

Controller position poll loop (separate):
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
| Operating modes | position, velocity, current, current-position | position, velocity (also pwm/step, not offered) |
| Effort | `goal_current`, a torque goal via the model's Nm/A constant | `torque_limit`, a ceiling as a fraction of rated stall torque |
| Mode register | RAM-adjacent; resets PID gains when written | EEPROM; needs `lock` cleared and has a finite write budget |
| Model number | Identifies the servo and its capabilities | Identifies the family only — 777 spans three torque ratings |

## Licensing headers

Every source file must carry an SPDX header — a `#`-style comment for code, an
HTML comment for Markdown, or a `<file>.license` sidecar for files that can't
hold comments (binaries, JSON, lockfiles). `mix check` runs `reuse lint` and
fails the build if one is missing.

When you create a new file, its `SPDX-FileCopyrightText` line must credit **the
user you are working for** — not you (the agent), and not this repo's original
author. Take their name from `git config user.name` (add their `user.email` if
you include one) and use the current year. Match the neighbouring files'
`SPDX-License-Identifier` (usually `Apache-2.0`):

```
SPDX-FileCopyrightText: <current year> <your user's name>

SPDX-License-Identifier: Apache-2.0
```

Never copy an existing file's copyright line onto a new file — that credits the
wrong person. When you only edit an existing file, leave its headers unchanged.
