# ALFA Robot PLC Driver

`alfa_robot_plc_driver` is the ROS-agnostic PLC Communication Core for the ALFA robot PLC/Modbus interface. It owns Modbus TCP communication, register encoding, PLC Axis address calculation, and safe command helpers.

## Verified protocol

- PLC is a Modbus TCP slave on `192.168.1.88:502`.
- Unit ID is `255`.
- Holding registers use 0-based Modbus addresses.
- `MB_CMD` base is `0`, `MB_STS` base is `1000`, `MB_SYS` base is `1960`.
- Each PLC Axis uses `32 WORD`.
- Axis1-6 are the currently connected arm.
- Axis7-12 are reserved for a second similar arm and are refused unless PLC reports them active.
- Positions are signed `DINT * 100`, low word first.
- Velocity/acceleration/deceleration/emergency deceleration are `WORD * 100`.
- MoveAbs is triggered by writing target/velocity/acc/dec/CommandID, then `ControlWord=5`.

## StatusWord note

`StatusWord` is read and exposed as raw data, but it is not used for control decisions yet. Real tests showed `AckCommandID`, `FeedbackPos`, `FeedbackSingle`, and `LastTarget` update correctly while `StatusWord` may remain `0x0000`.

If PLC documentation says Axis1 status is `0..31`, treat that as the offset inside `MB_STS`. The Modbus holding-register address for `MB_STS[0]` is still `1000` when `MB_STS AT %MW1000`.

## CLI

```bash
cd tools/alfa_robot_plc_driver
python3 -m alfa_robot_plc_driver.cli status
python3 -m alfa_robot_plc_driver.cli read-angles
python3 -m alfa_robot_plc_driver.cli move-delta --deltas 1:0.1,2:0.1 --vel 3 --yes-write
python3 -m alfa_robot_plc_driver.cli clear --all --yes-write
```

Mock mode:

```bash
python3 -m alfa_robot_plc_driver.cli --mock status
python3 -m alfa_robot_plc_driver.cli --mock move-delta --deltas 1:0.1,2:0.1 --yes-write
```
