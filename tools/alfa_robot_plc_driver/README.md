# ALFA Robot PLC Driver Test Package

`alfa_robot_plc_driver` is a non-ROS Python test package for the PLC/Modbus motion-control interface. It validates the 12-axis communication model before the interface is wrapped as a ROS2 node or `ros2_control` hardware plugin.

## Scope

- Models the dual-arm robot as 12 homogeneous PLC axes.
- Uses `axis1..axis6 -> left_v5_joint1..left_v5_joint6` and `axis7..axis12 -> right_v5_joint1..right_v5_joint6` by default.
- Supports mock mode while the final InoProShop Modbus map is missing.
- Keeps all Coil/Holding Register addresses in YAML instead of scattering raw addresses through code.

## Install for Local Testing

```bash
cd tools/alfa_robot_plc_driver
python3 -m pip install -e '.[test]'
```

## Mock Examples

```bash
python3 -m alfa_robot_plc_driver.cli --mock status
python3 -m alfa_robot_plc_driver.cli --mock enable --axis 1
python3 -m alfa_robot_plc_driver.cli --mock move --axis 1 --deg 30
python3 -m alfa_robot_plc_driver.cli --mock move-many --targets '1:10,2:20,7:-15'
python3 -m alfa_robot_plc_driver.cli --mock emergency-stop --all
```

`move-many` is not trajectory interpolation. It quickly writes separate target positions for the selected axes and waits for each axis command ID to complete. If any selected axis fails or times out, the default stop policy sends `StopExecute` to all participating axes.

## Filling the Real PLC Map

When the PLC owner provides the final InoProShop Modbus mapping, fill `config/plc_modbus_map.example.yaml` or copy it to a deployment-specific file and replace:

- `axes.axisN.coils.*` with actual Coil addresses.
- `axes.axisN.registers.*.address` with actual Holding Register start addresses.
- `encoding.lreal_word_order` and `encoding.udint_word_order` with the verified register word order.

Real PLC mode rejects `null` addresses or `unknown` word order. This is intentional to avoid writing to an incorrect PLC variable.

## Real PLC Example

```bash
python3 -m alfa_robot_plc_driver.cli --config config/plc_modbus_map.real.yaml status
```

Do not use real PLC mode until the Modbus map is confirmed to match the PLC program currently downloaded to the controller.
