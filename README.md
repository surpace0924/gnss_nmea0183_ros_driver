# gnss_nmea0183_ros_driver
GNSS ROS driver in nmea0183 format



## Inputs / Outputs

### Output

| Name              | Type                              | Description            |
| ----------------- | --------------------------------- | ---------------------- |
| `~/gnss_fix`      | `sensor_msgs/NavSatFix`           | GNSS data              |
| `~/diagnostics`   | `diagnostic_msgs/DiagnosticArray` | GNSS connection status |

## Parameters

### Node Parameters

| Name                     | Type   | Default Value  | Description                                         |
| ------------------------ | ------ | -------------- | --------------------------------------------------- |
| `timeout`                | float  | 5.0            | timeout of serial communication [sec]               |
| `baudrate`               | int    | 9600           | serial baundrate [bps]                              |
| `port`                   | string | "/dev/ttyUSB0" | USB port to which GNSS is connected                 |
| `frame_id`               | string | "base_link"    | GNSS frame id                                       |
| `sweep_rate`             | float  | 10.0           | Serial data extraction rate [Hz]                    |
| `diagnostic_status_name` | string | "GNSS"         | status_name of diagnostic msg                       |
| `dummy_latitude`         | float  | 35.689         | latitude for dummy (dummy node only)                |
| `dummy_longitude`        | float  | 139.692        | longitude for dummy (dummy node only)               |
| `dummy_altitude`         | float  | 40.0           | altitude for dummy (dummy node only)                |
| `publish_rate`           | float  | 1.0            | dummy GNSS data publish rate [Hz] (dummy node only) |


## Software Requirement
- Python3
- pynmea2
- serial


## Operation confirmed environment
- OS: Ubuntu20.04
- ROS: ROS noetic
- GNSS: GU-502MGG-USB

## Usage
### 1. Please connect GNSS via USB

### 2. Install requirement package
```
pip install pynmea2
pip install pyserial
```

### 3. Launch the node
```
pip install pynmea2
pip install pyserial
```
