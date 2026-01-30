# Gimbal Step-by-Step Auto-Tester

Automatically tests all gimbal binary-protocol commands in order. Use after flashing the ESP32 firmware once; no re-flash needed between test runs.

## Setup

1. Flash the ESP32 with the pan_tilt firmware (binary protocol) once.
2. From this folder, install deps: `pip install -r requirements.txt`

## Run

From `tester/`:

```bash
python run_tests.py COM3
```

(Use your actual port: e.g. `COM4`, `/dev/ttyUSB0` on Linux.)

Optional:

- `python run_tests.py COM3 --baud 921600` (default baud is 921600)
- `python run_tests.py COM3 --run-destructive` to run stage 10 (SET_SERVO_ID, CALIBRATE); **skip on production hardware**

## Output

- Per-stage: `Stage N: Name ... PASS` or `Stage N: Name ... FAIL <reason>`
- Final: `Result: X/Y passed` (exit code 0 if all passed, 1 otherwise)

## Stages

| Stage | Name                    | Commands                          |
|-------|-------------------------|-----------------------------------|
| 1     | Connection + GET_IMU    | 126                               |
| 2     | GET_INA                 | 160                               |
| 3     | State machine           | 137, 135                          |
| 4     | Move commands           | 133, 134, 172, 173, 174, 175      |
| 5     | Stop and lock           | 135, 170, 171                     |
| 6     | User + settings         | 141, 131, 142, 136                |
| 7     | CONFIG + PING           | 139, 200 (id 1, 2)                |
| 8     | Register R/W             | 210, 212, 211, 213                |
| 9     | EXIT_CONFIG + TRACKING  | 140, 137                          |
| 10    | (optional) Destructive  | 501, 502 — use `--run-destructive` |

All implementation files for the auto-tester live in this folder (`tester/`).
