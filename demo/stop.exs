# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

# Cuts torque on every servo on the bus, and stops anything still turning.
#
#     BB_VERSION=local mix run demo/stop.exs
#
# Use this if a demo is interrupted partway through: a servo in velocity mode
# keeps spinning at whatever it was last told, and killing the VM does not
# change that.

port = System.get_env("PORT", "/dev/ttyUSB0")
ids = 1..(System.get_env("MAX_ID", "6") |> String.to_integer())

{:ok, pid} = Feetech.start_link(port: port, baud_rate: 1_000_000)

for id <- ids do
  Feetech.write_raw(pid, id, :goal_speed, 0, await_response: true)
  Feetech.write_raw(pid, id, :torque_enable, 0, await_response: true)
end

Process.sleep(200)

for id <- ids do
  case Feetech.read_raw(pid, id, :torque_enable) do
    {:ok, 0} -> IO.puts("servo #{id}: passive")
    {:ok, other} -> IO.puts("servo #{id}: STILL POWERED (torque_enable=#{other})")
    {:error, reason} -> IO.puts("servo #{id}: no response (#{inspect(reason)})")
  end
end

Feetech.stop(pid)
