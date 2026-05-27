# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoFeetech.Install do
    @shortdoc "Installs BB.Servo.Feetech into a robot"
    @moduledoc """
    #{@shortdoc}

    Adds a `BB.Servo.Feetech.Controller` and `BB.Servo.Feetech.Bridge` to your
    robot module, defines a `:config.:feetech` param group, writes the serial
    device default to `config/config.exs`, and wires the robot's child spec to
    load its opts from the application environment.

    Actuators belong on individual joints in your topology and are not added
    automatically — a snippet is printed for you to copy.

    ## Example

    ```bash
    mix igniter.install bb_servo_feetech
    mix igniter.install bb_servo_feetech --robot MyApp.Arm
    mix igniter.install bb_servo_feetech --device /dev/ttyACM0
    ```

    ## Options

    * `--robot` - The robot module (defaults to `{AppPrefix}.Robot`).
    * `--name` - The controller name (default `feetech`).
    * `--bridge-name` - The parameter bridge name (default `feetech_bridge`).
    * `--device` - The serial device path (default `/dev/ttyUSB0`).
    """

    use Igniter.Mix.Task

    alias Igniter.Code.{Common, Function, Tuple}
    alias Igniter.Project.Formatter

    @controller_module BB.Servo.Feetech.Controller
    @param_group :feetech
    @default_device "/dev/ttyUSB0"

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [
          robot: :string,
          name: :string,
          bridge_name: :string,
          device: :string
        ],
        aliases: [r: :robot, n: :name]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options
      robot_module = BB.Igniter.robot_module(igniter)
      name = options |> Keyword.get(:name, "feetech") |> String.to_atom()
      bridge_name = options |> Keyword.get(:bridge_name, "feetech_bridge") |> String.to_atom()
      device = Keyword.get(options, :device, @default_device)

      case existing_controller_name(igniter, robot_module) do
        {:ok, existing_name} ->
          Igniter.add_notice(igniter, already_installed_notice(robot_module, existing_name))

        :error ->
          igniter
          |> Formatter.import_dep(:bb_servo_feetech)
          |> BB.Igniter.add_controller(robot_module, name, controller_code(name))
          |> BB.Igniter.add_parameter_bridge(
            robot_module,
            bridge_name,
            bridge_code(bridge_name, name)
          )
          |> BB.Igniter.add_param_group(robot_module, [:config, @param_group], param_group_body())
          |> BB.Igniter.set_robot_param_default(
            robot_module,
            [:config, @param_group, :device],
            device
          )
          |> Igniter.add_notice(topology_snippet(name))
      end
    end

    # The Feetech bus is singleton-per-robot. If a controller targeting
    # `BB.Servo.Feetech.Controller` is already declared, return its name so the
    # caller can short-circuit instead of adding a duplicate under a different
    # entry name (e.g. when both `bb_servo_feetech` and a higher-level
    # composer like `bb_so101` appear in `mix igniter.new --install`).
    defp existing_controller_name(igniter, robot_module) do
      with {:ok, {_igniter, _source, zipper}} <-
             Igniter.Project.Module.find_module(igniter, robot_module),
           {:ok, zipper} <- Common.move_to_do_block(zipper),
           {:ok, zipper} <-
             Function.move_to_function_call_in_current_scope(zipper, :controllers, 1),
           {:ok, zipper} <- Common.move_to_do_block(zipper),
           {:ok, call_zipper} <-
             Function.move_to_function_call_in_current_scope(
               zipper,
               :controller,
               [2, 3],
               &controller_targets_module?/1
             ),
           {:ok, name_zipper} <- Function.move_to_nth_argument(call_zipper, 0),
           name when is_atom(name) <- extract_atom(name_zipper) do
        {:ok, name}
      else
        _ -> :error
      end
    end

    defp controller_targets_module?(call_zipper) do
      case Function.move_to_nth_argument(call_zipper, 1) do
        {:ok, arg_zipper} -> Tuple.elem_equals?(arg_zipper, 0, @controller_module)
        _ -> false
      end
    end

    defp extract_atom(zipper) do
      case Sourceror.Zipper.node(zipper) do
        atom when is_atom(atom) -> atom
        _ -> nil
      end
    end

    defp controller_code(name) do
      """
      controller :#{name}, {BB.Servo.Feetech.Controller,
        port: param([:config, :#{@param_group}, :device]),
        baud_rate: param([:config, :#{@param_group}, :baud_rate]),
        control_table: Feetech.ControlTable.STS3215}
      """
    end

    defp bridge_code(bridge_name, controller_name) do
      "bridge :#{bridge_name}, {BB.Servo.Feetech.Bridge, controller: :#{controller_name}}\n"
    end

    defp param_group_body do
      """
      param :device, type: :string, doc: "Serial device connected to the Feetech servo bus"

      param :baud_rate,
        type: :integer,
        default: 1_000_000,
        doc: "Communications speed for the serial port"
      """
    end

    defp already_installed_notice(robot_module, existing_name) do
      """
      bb_servo_feetech: #{inspect(robot_module)} already has a #{inspect(@controller_module)} \
      registered as `:#{existing_name}`. Skipping install — the Feetech bus is singleton per \
      robot. If you intended to add servos, attach actuators to joints referencing \
      `controller: :#{existing_name}` instead.
      """
    end

    defp topology_snippet(controller_name) do
      """
      bb_servo_feetech: add servo actuators to your joints. Example:

          joint :shoulder, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.Feetech.Actuator,
              servo_id: 1,
              controller: :#{controller_name}}
          end
      """
    end
  end
else
  defmodule Mix.Tasks.BbServoFeetech.Install do
    @shortdoc "Installs BB.Servo.Feetech into a robot"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_servo_feetech.install task requires igniter.

          mix igniter.install bb_servo_feetech
      """)

      exit({:shutdown, 1})
    end
  end
end
