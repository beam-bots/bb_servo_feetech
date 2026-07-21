<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.Servo.Feetech Usage Rules

`bb_servo_feetech` drives Feetech/WaveShare STS/SCS serial-bus servos for
[Beam Bots](https://hexdocs.pm/bb). It ships three components: a
`BB.Servo.Feetech.Controller` (a `BB.Controller`, one per serial bus),
`BB.Servo.Feetech.Actuator` (a `BB.Actuator`, one per servo), and an optional
`BB.Servo.Feetech.Bridge` (a `BB.Bridge`, exposing servo control-table
parameters). For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
Feetech.

## Core principles

1. **One controller per serial bus, shared by every servo on it.** The
   controller owns the serial connection and runs a fixed-rate loop that batches
   pending commands into `sync_write` and reads positions via `sync_read`.
   Actuators never touch the port — they reference the controller by its DSL
   name and write into its shared command table. The bus is
   singleton-per-robot.
2. **Servos are closed-loop; no separate sensor is needed.** The controller
   polls `present_position` and publishes `BB.Message.Sensor.JointState` per
   joint, plus `BB.Servo.Feetech.Message.ServoStatus` (temperature, voltage,
   load, hardware error).
3. **Command joint-space; the transmission handles motor-space.** Set positions
   in radians via `BB.Actuator`; BB applies the joint's `transmission`
   (`reversed?`, `offset`, gearing) before the value reaches the driver.
4. **Torque is centralised in the controller.** On arm it enables torque on all
   registered servos; on disarm or crash it disables (or holds) torque for every
   servo ID in one pass. The actuator's `disarm/1` is a deliberate no-op.

## Wiring it into the DSL

Declare the controller in a `controllers` block, attach an actuator to each
driven joint, and optionally add the parameter bridge under `parameters`
(verified against `bb_example_so101`):

```elixir
controllers do
  controller :feetech_controller,
    {BB.Servo.Feetech.Controller,
     port: param([:config, :feetech, :device]),
     baud_rate: 1_000_000,
     control_table: Feetech.ControlTable.STS3215,
     disarm_action: :hold},
    simulation: :omit
end

topology do
  link :base_link do
    joint :shoulder_pan do
      type :revolute

      limit do
        lower ~u(-110 degree)
        upper ~u(110 degree)
        velocity ~u(360 degree_per_second)
        effort ~u(2.5 newton_meter)
      end

      actuator :shoulder_pan_servo,
               {BB.Servo.Feetech.Actuator, servo_id: 1, controller: :feetech_controller} do
        transmission do
          reversed? true
        end
      end

      # ... nested links/joints, one actuator per servo
    end
  end
end

parameters do
  bridge :feetech_bridge, {BB.Servo.Feetech.Bridge, controller: :feetech_controller},
    simulation: :mock
end
```

## Options

Controller (`{BB.Servo.Feetech.Controller, opts}`):

| Option | Default | Meaning |
|---|---|---|
| `:port` | required | Serial device, e.g. `"/dev/ttyUSB0"` |
| `:baud_rate` | `1_000_000` | Bus speed, bps |
| `:control_table` | `Feetech.ControlTable.STS3215` | Servo control table |
| `:loop_interval_ms` | `10` | Control-loop period (100 Hz) |
| `:status_poll_interval_ms` | `1000` | Status poll period; `0` disables |
| `:disarm_action` | `:disable_torque` | `:disable_torque` or `:hold` |

Actuator (`{BB.Servo.Feetech.Actuator, opts}`):

| Option | Default | Meaning |
|---|---|---|
| `:servo_id` | required | Servo address on the bus, `1..253` |
| `:controller` | required | Name of the controller entry in the DSL |
| `:position_deadband` | `2` | Min raw-unit change before publishing feedback |

Position and velocity limits come from the joint's `limit` block, not actuator
options — the actuator derives its motor profile from the topology.

## Anti-patterns

- **Don't declare a controller per servo, or give actuators a `port`.** One
  controller per bus; each actuator on it sets `controller:` to that entry's
  name and `servo_id:` to its address.
- **Don't pass `reversed?`/`offset` as actuator options.** Direction and
  zero-offset live in the joint's `transmission do … end` block; the actuator
  schema accepts only `servo_id`, `controller`, and `position_deadband`.
- **Don't expect a servo to cut its own torque on disarm.** `Actuator.disarm/1`
  is a no-op; the controller disables torque for all servo IDs. Note that
  `disarm_action: :hold` keeps torque *on* — it holds position, it does not
  power down.
- **Don't assume the controller runs in simulation.** Controllers default to
  `simulation: :omit`, and actuators are auto-swapped for `BB.Sim.Actuator`. Set
  the controller to `simulation: :mock`/`:start` only if you need the real bus
  under simulation.

## Further reading

- [bb_servo_feetech docs](https://hexdocs.pm/bb_servo_feetech)
- `bb`'s actuator and safety rules (`bb:actuators`, `bb:safety-and-commands`)
  and [Parameter Bridges](https://hexdocs.pm/bb/08-parameter-bridges.html)
