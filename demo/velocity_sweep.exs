# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# Drives one servo through a range of speeds in both directions, printing what
# was commanded next to what the encoder actually did.
#
#     BB_VERSION=local mix run demo/velocity_sweep.exs
#
# The servo needs to be free to rotate continuously — take the horn off first.
# Set SERVO_ID and PORT to match your bus.
#
# If you interrupt this partway through, the servo keeps spinning at whatever
# it was last told. `mix run demo/stop.exs` cuts torque on the whole bus.

port = System.get_env("PORT", "/dev/ttyUSB0")
servo_id = System.get_env("SERVO_ID", "6") |> String.to_integer()

defmodule SweepBot do
  use BB

  @port System.get_env("PORT", "/dev/ttyUSB0")
  @servo_id System.get_env("SERVO_ID", "6") |> String.to_integer()

  controllers do
    controller(
      :bus,
      {BB.Servo.Feetech.Controller,
       port: @port,
       baud_rate: 1_000_000,
       control_table: Feetech.ControlTable.STS3215,
       disarm_action: :disable_torque}
    )
  end

  commands do
    command :arm do
      handler(BB.Command.Arm)
      allowed_states([:disarmed])
    end

    command :disarm do
      handler(BB.Command.Disarm)
      allowed_states([:idle])
    end
  end

  topology do
    link :base do
      joint :spinner do
        type(:revolute)

        limit do
          lower(~u(-170 degree))
          upper(~u(170 degree))
          effort(~u(2.5 newton_meter))
          velocity(~u(333 degree_per_second))
          acceleration(~u(439 degree_per_square_second))
        end

        actuator(
          :spinner_servo,
          {BB.Servo.Feetech.Actuator, servo_id: @servo_id, controller: :bus, mode: :velocity}
        )

        link :tip do
        end
      end
    end
  end
end

defmodule Sweep do
  @steps_per_rev 4096
  @sample_ms 100

  def read(id, reg) do
    {:ok, v} = BB.Process.call(SweepBot, :bus, {:read_raw, id, reg})
    v
  end

  # Sample often enough that a wrap is never ambiguous: even at full tilt the
  # servo covers well under half a revolution between samples.
  def measure(id, duration_ms) do
    samples = div(duration_ms, @sample_ms)

    {total, _last} =
      Enum.reduce(1..samples, {0, read(id, :present_position)}, fn _, {acc, previous} ->
        Process.sleep(@sample_ms)
        current = read(id, :present_position)
        {acc + unwrap(current - previous), current}
      end)

    total / (samples * @sample_ms / 1000) * 360 / @steps_per_rev
  end

  defp unwrap(delta) when delta > 2048, do: delta - @steps_per_rev
  defp unwrap(delta) when delta < -2048, do: delta + @steps_per_rev
  defp unwrap(delta), do: delta

  def spin(id, rad_per_s, hold_ms \\ 2000) do
    BB.Actuator.set_velocity(SweepBot, :spinner_servo, rad_per_s)
    # let it come up to speed before measuring
    Process.sleep(400)
    raw = read(id, :goal_speed)
    measured = measure(id, hold_ms)

    commanded_deg = rad_per_s * 180 / :math.pi()

    IO.puts([
      String.pad_leading(:erlang.float_to_binary(rad_per_s, decimals: 2), 6),
      " rad/s  │ ",
      String.pad_leading(:erlang.float_to_binary(commanded_deg, decimals: 0), 5),
      "°/s  │ raw ",
      String.pad_leading(to_string(raw), 6),
      " │ measured ",
      String.pad_leading(:erlang.float_to_binary(measured, decimals: 0), 5),
      "°/s  ",
      bar(measured)
    ])
  end

  defp bar(deg_per_s) do
    width = round(abs(deg_per_s) / 12)
    arrow = if deg_per_s < 0, do: "<", else: ">"
    String.duplicate(arrow, min(width, 30))
  end

  def pause(id) do
    BB.Actuator.set_velocity(SweepBot, :spinner_servo, 0.0)
    Process.sleep(700)
    read(id, :present_position)
  end
end

IO.puts("""

Velocity sweep — servo #{servo_id} on #{port}
Make sure the output shaft is free to turn continuously.
""")

{:ok, _} = SweepBot.start_link([])
Process.sleep(1200)

IO.puts("mode register: #{Sweep.read(servo_id, :mode)} (1 = velocity)")

{:ok, cmd} = SweepBot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)
Process.sleep(300)
IO.puts("armed\n")

IO.puts("commanded    │ commanded │ goal_speed │ measured")
IO.puts(String.duplicate("─", 78))

speeds = [0.25, 0.5, 1.0, 2.0, 4.0, 5.8]

for s <- speeds do
  Sweep.spin(servo_id, s)
  Sweep.pause(servo_id)
end

IO.puts("")

for s <- speeds do
  Sweep.spin(servo_id, -s)
  Sweep.pause(servo_id)
end

IO.puts("""

The last rows should fall short of what was asked: the joint is declared at
333°/s and the servo tops out near there, so commanding more just saturates.
Negative speeds are encoded sign-magnitude — 'raw' passes 32768 rather than
going negative.
""")

IO.puts("── duration: the command stops itself after 1.5s ──")
BB.Actuator.set_velocity(SweepBot, :spinner_servo, 2.0, duration: 1500)

for t <- 1..6 do
  Process.sleep(500)

  IO.puts(
    "  t+#{t * 500}ms  goal_speed=#{Sweep.read(servo_id, :goal_speed)}  torque=#{Sweep.read(servo_id, :torque_enable)}"
  )
end

IO.puts("""

  expiry_action defaults to :stop, so torque was cut when the duration ran out
  and the shaft is now free. `:hold` would have kept it powered at a standstill.
""")

IO.puts("── hold: powered, resisting rotation — try turning it by hand ──")
BB.Actuator.hold(SweepBot, :spinner_servo)
Process.sleep(300)

IO.puts(
  "  torque=#{Sweep.read(servo_id, :torque_enable)}  goal_speed=#{Sweep.read(servo_id, :goal_speed)}"
)

Process.sleep(5000)

IO.puts("\n── stop: passive, free to turn — try again ──")
BB.Actuator.stop(SweepBot, :spinner_servo)
Process.sleep(300)
IO.puts("  torque=#{Sweep.read(servo_id, :torque_enable)}")
Process.sleep(5000)

{:ok, d} = SweepBot.disarm()
BB.Command.await(d)
IO.puts("\ndisarmed, torque=#{Sweep.read(servo_id, :torque_enable)}")
IO.puts("servo left in velocity mode — starting a position-mode robot puts it back.")
