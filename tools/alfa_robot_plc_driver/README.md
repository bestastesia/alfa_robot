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
python3 -m alfa_robot_plc_driver.cli smoke-12 --delta 0.1 --vel 3 --yes-write
python3 -m alfa_robot_plc_driver.cli stream-abs --axes 1 --speed 0.5 --duration-s 2 --hz 5 --feedback-hz 2 --vel 30 --max-delta 2 --yes-write
python3 -m alfa_robot_plc_driver.cli estop --axes 1 --yes-write
python3 -m alfa_robot_plc_driver.cli reset-estop --axes 1 --yes-write
python3 -m alfa_robot_plc_driver.cli clear --all --yes-write
```

If the laptop routes `192.168.1.88` through Wi-Fi, pin the PLC route to the wired port first:

```bash
tools/alfa_robot_plc_driver/scripts/plc_net_setup.sh
```

Mock mode:

```bash
python3 -m alfa_robot_plc_driver.cli --mock status
python3 -m alfa_robot_plc_driver.cli --mock move-delta --deltas 1:0.1,2:0.1 --yes-write
python3 -m alfa_robot_plc_driver.cli --mock --mock-active-axes 12 smoke-12 --axes 12 --delta 0.1 --yes-write
python3 -m alfa_robot_plc_driver.cli --mock --mock-active-axes 12 stream-abs --axes 1,2 --speed 0.5 --duration-s 0.2 --hz 5 --feedback-hz 5 --max-delta 1 --vel 20 --yes-write
```

`stream-abs` is only an experiment for low-frequency absolute target streaming. It sets a high PLC profile velocity and repeatedly sends new absolute targets at `--hz`; it keeps one TCP connection open and reads feedback at `--feedback-hz` to avoid disturbing the write cadence. It is not a hard real-time servo loop.

Emergency stop commands write only the per-axis control word:

- `estop`: `ControlWord=257 / 0x0101 = Enable + EmergencyStop`
- `reset-estop`: `ControlWord=1025 / 0x0401 = Enable + ResetEmergency`
- `reset-fault`: `ControlWord=3 / 0x0003 = Enable + ResetFault`
