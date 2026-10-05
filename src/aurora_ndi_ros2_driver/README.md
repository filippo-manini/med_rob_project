# Aurora NDI ROS2 Driver

[![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)

ROS2 driver for NDI Aurora electromagnetic tracking with advanced filtering and analysis tools.

## Features

- **Real-time Tracking**: Up to 40 Hz pose tracking with multi-sensor support (up to 4)
- **Advanced Filtering**: Kalman filter and low-pass filter for improved accuracy
- **Calibration Tools**: Noise estimator for automatic filter tuning
- **Analysis Tools**: Delay analyzer for latency measurement
- **TF2 Integration**: Automatic transform broadcasting
- **Flexible Configuration**: YAML-based configuration with autoconfigured tools support

## Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/eddrive/aurora_ndi_ros2_driver.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select aurora_ndi_ros2_driver
source install/setup.bash
```

### Device Permissions

```bash
sudo usermod -a -G dialout $USER  # Log out and back in
```

### Basic Usage

```bash
# Launch Aurora tracking with filters
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py

# With RViz visualization
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py use_rviz:=true
```

## Prerequisites

- **Hardware**: NDI Aurora system with USB connection (`/dev/ttyUSB0`)
- **Software**: Ubuntu 22.04 + ROS2 Humble
- **Dependencies**: `libserial-dev`, `ros-humble-tf2-ros`, `ros-humble-geometry-msgs`

## Package Structure

```
aurora_ndi_ros2_driver/
├── config/
│   ├── driver/                     # Main tracking configuration
│   └── analysis/                   # Calibration and analysis configs
├── launch/
│   ├── driver/                     # Main launch files
│   │   └── aurora_tracking.launch.py
│   └── analysis/                   # Analysis tools
│       ├── aurora_noise_calibration.launch.py
│       └── delay_analyzer.launch.py
├── src/
│   ├── core/                       # Aurora communication
│   ├── filters/                    # Kalman and low-pass filters
│   ├── nodes/                      # ROS2 nodes
│   └── utils/                      # Utilities
├── msg/
│   └── AuroraData.msg              # Position, orientation, error
└── rom/                            # Sensor ROM files
```

## Configuration

Main configuration file: [config/driver/aurora_tracking_config.yaml](config/driver/aurora_tracking_config.yaml)

### Multi-Sensor Setup

```yaml
num_sensors: 2                      # Total sensors (1-4)
tool_rom_files: ["sensor.rom"]      # ROM files (empty "" for autoconfigured)
port_handles: ["0A", "0B"]          # All port handles
port_handles_autoconfig: ["0B"]     # Autoconfigured ports (NDI pens)
```

**Rule**: `len(tool_rom_files) = num_sensors - len(port_handles_autoconfig)`

### Filter Configuration

```yaml
# Enable/disable filters
enable_kalman_filter: true
enable_lowpass_filter: false
```

Filters are configured in separate files:
- [config/driver/kalman_filter_config.yaml](config/driver/kalman_filter_config.yaml)
- [config/driver/lowpass_filter_config.yaml](config/driver/lowpass_filter_config.yaml)

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Aurora device path |
| `baud_rate` | `230400` | Communication speed |
| `publish_rate_hz` | `40.0` | Publishing frequency |
| `frame_id` | `aurora_base` | Parent TF frame |

## Usage

### Main Tracking

```bash
# Basic tracking
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py

# With visualization
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py use_rviz:=true

# Debug mode
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py debug:=true log_level:=debug
```

### Noise Calibration

Calibrate Kalman filter parameters (sensor must be stationary):

```bash
# Start tracking first
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py

# In another terminal, run calibration
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py max_samples:=1000
```

Output: `noise_calibration_results.yaml` with recommended filter parameters.

### Delay Analysis

Measure system latency:

```bash
ros2 launch aurora_ndi_ros2_driver delay_analyzer.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `aurora_tracking_config.yaml` | Main configuration |
| `use_rviz` | `false` | Launch RViz |
| `debug` | `false` | Debug logging |
| `log_level` | `info` | Log verbosity |

### Monitoring

```bash
# Topic data
ros2 topic echo /aurora/sensor0
ros2 topic hz /aurora/sensor0

# Transforms
ros2 run tf2_ros tf2_echo aurora_base endo_aurora_sensor0
```

## ROS2 Interface

### Topics

**Raw Data** (configured in YAML):
- `/aurora/sensor0` - `aurora_ndi_ros2_driver/AuroraData`
- `/aurora/sensor1` - (if multi-sensor setup)

**Filtered Data** (if filters enabled):
- `/aurora/sensor0/kalman` - Kalman-filtered output
- `/aurora/sensor0/lowpass` - Low-pass filtered output

### Message: AuroraData

```
Header header                       # Timestamp + frame_id
Point position                      # Position in mm
Quaternion orientation              # Orientation (x,y,z,w)
float64 error                       # RMS error in mm
bool visible                        # Sensor visibility
uint32 port_handle                  # Port identifier
```

### TF Tree

```
world
  └── aurora_base
      ├── endo_aurora_sensor0
      └── endo_aurora_sensor1 (if configured)
```

Published at `publish_rate_hz` (default 40 Hz)

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| **Serial port access denied** | `sudo usermod -a -G dialout $USER` (log out/in) |
| **Device not found** | Check `ls -l /dev/ttyUSB*` and config file path |
| **Sensor not visible** | Check tracking volume, remove metal objects |
| **ROM file mismatch** | Verify ROM files match: `num_sensors - num_autoconfig` |
| **Low publishing rate** | Disable unused filters, check CPU usage |
| **Build errors** | `rosdep install --from-paths src --ignore-src -r -y` |

### Debug Mode

```bash
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py debug:=true log_level:=debug
```

## Advanced Features

### Kalman Filter
Provides optimal state estimation with process and measurement noise modeling. Configure via [kalman_filter_config.yaml](config/driver/kalman_filter_config.yaml).

### Low-Pass Filter
Reduces high-frequency noise for smoother trajectories. Configure via [lowpass_filter_config.yaml](config/driver/lowpass_filter_config.yaml).

### Noise Estimator
Automatically computes optimal Kalman filter parameters from stationary sensor data.

### Delay Analyzer
Measures end-to-end system latency for time-critical applications.

## License & Acknowledgments

**Copyright © 2024 Edoardo Guida**
Developed at [NEARlab, Politecnico di Milano](https://nearlab.polimi.it/)

Contact: [edoardo1.guida@mail.polimi.it](mailto:edoardo1.guida@mail.polimi.it)

---

**Docker deployment**: See [`docker-setup`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/docker-setup) branch
**Issues**: [GitHub Issues](https://github.com/eddrive/aurora_ndi_ros2_driver/issues)