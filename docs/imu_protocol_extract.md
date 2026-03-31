# IMU Protocol Extract

## Module Identity

`ATK-IMU901` and `ATK-MS901M` refer to the same module family for this project. The implementation uses the `ATK-MS901M` protocol documents and sample code.

## Transport

- Physical transport: UART
- Connected interface: `UART0`
- Default baud rate: `115200`

## Frame Structure

- Upload frame header: `0x55 0x55`
- Register / ACK frame header: `0x55 0xAF`
- Checksum: byte sum over the frame bytes before the checksum field

## Upload Frame IDs

| ID | Meaning |
|---|---|
| `0x01` | Attitude |
| `0x02` | Quaternion |
| `0x03` | Gyro + Accelerometer |
| `0x04` | Magnetometer |
| `0x05` | Barometer |
| `0x06` | Port state |

## Relevant Register IDs

| ID | Meaning |
|---|---|
| `0x07` | UART baud rate |
| `0x08` | Return set |
| `0x0A` | Return rate |
| `0x0B` | Algorithm select |
| `0x0C` | Installation orientation |

## Confirmed Return-Rate Table

| Code | Rate |
|---|---|
| `0x00` | `250 Hz` |
| `0x01` | `200 Hz` |
| `0x02` | `125 Hz` |
| `0x03` | `100 Hz` |
| `0x04` | `50 Hz` |
| `0x05` | `20 Hz` |
| `0x06` | `10 Hz` |
| `0x07` | `5 Hz` |
| `0x08` | `2 Hz` |
| `0x09` | `1 Hz` |

## Firmware Mode Split

### Mode A: Raw Sensor Mode

- Default return content: `gyro + acc`
- Optional return content: `mag`
- Disabled by default: `baro`, `attitude`, `quaternion`
- ESP32 side responsibilities: timestamping, coordinate mapping, gyro bias calibration, quaternion estimation, Euler output, health output

### Mode B: Direct Attitude Mode

- Default return content: `attitude + quaternion`
- Optional raw frames: off by default
- ESP32 side responsibilities: frame parsing, coordinate mapping, validity filtering, timeout detection, outlier rejection, health output

## Unified Output Contract

Every upper layer consumes the same `imu_sample_t`:

- `timestamp_us`
- `gyro_xyz_dps`
- `acc_xyz_g`
- `quat_wxyz`
- `roll_pitch_yaw_deg`
- `health`
- `update_age_us`

## Runtime Design Constraint

The module only actively uploads at up to `250 Hz`. The control architecture must therefore use:

- `1 kHz` motor/output scheduling
- estimator update only on fresh IMU samples
- rate PID only on fresh IMU samples
- lower-rate angle PID

The firmware must not fake `1 kHz` sensor updates by repeatedly re-processing stale IMU data.
