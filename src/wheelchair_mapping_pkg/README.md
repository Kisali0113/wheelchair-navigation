# Wheelchair Mapping Package

A ROS2 Humble package for real-world 2D LiDAR SLAM mapping on a three-wheel front-drive, front-steer wheelchair platform.

## Package Contents

- `wheelchair_mapping_pkg/serial_sensor_node.py`
  - Reads Arduino serial data from a 2D LiDAR wheelchair.
  - Publishes `/steerticks`, `/wheelticks`, and `/imu/data`.
- `wheelchair_mapping_pkg/encoder_odom_node.py`
  - Converts encoder ticks into odometry.
  - Publishes `/wheel/odom` and broadcasts `odom -> base_link`.
- `config/ekf.yaml`
  - `robot_localization` EKF configuration for indoor 2D fusion.
- `launch/mapping.launch.py`
  - Starts serial interface, odometry node, EKF, static TF, and SLAM Toolbox.

## Hardware Assumptions

- 2D LiDAR publishing `/scan`
- Arduino sending CSV serial lines in format:
  `left_ticks,right_ticks,yaw,gyro_z,acc_x,acc_y`
- Three-wheel front drive, front steer configuration
- Ubuntu + ROS2 Humble

## Supported Workflow

1. Run `serial_sensor_node` for Arduino bridge
2. Run `encoder_odom_node` for planar wheel odometry
3. Fuse odometry and IMU with `robot_localization` EKF
4. Launch SLAM Toolbox asynchronously for live mapping
5. Visualize map and TF in RViz2

## Launch

Build the package:

```bash
cd /home/janidu/fyp_ws
colcon build --packages-select wheelchair_mapping_pkg
source install/setup.bash
```

Start mapping:

```bash
ros2 launch wheelchair_mapping_pkg mapping.launch.py
```

### Optional launch overrides

```bash
ros2 launch wheelchair_mapping_pkg mapping.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200 wheel_radius:=0.05 wheel_base:=0.35 ticks_per_revolution:=1024 steer_ticks_per_revolution:=1024 steer_max_angle_deg:=30.0
```

## ROS Topics

### Published by `serial_sensor_node`

- `/steerticks` (`std_msgs/Int32`)
- `/wheelticks` (`std_msgs/Int32`)
- `/imu/data` (`sensor_msgs/Imu`)

### Published by `encoder_odom_node`

- `/wheel/odom` (`nav_msgs/Odometry`)

### Published by EKF

- `/odometry/filtered` (`nav_msgs/Odometry`)

### Consumed by SLAM Toolbox

- `/scan` (`sensor_msgs/LaserScan`)
- `map_frame`: `map`
- `odom_frame`: `odom`
- `base_frame`: `base_link`

## TF Tree

The package creates the following frames:

- `map`
- `odom`
- `base_link`
- `laser_frame`
- `imu_link`

Static transform:

- `base_link` -> `laser_frame`

EKF broadcasts:

- `odom` -> `base_link`

## RViz Setup

Use RViz2 with the following displays:

- Fixed Frame: `map`
- LaserScan: `/scan`
- Map
- TF
- Odometry: `/odometry/filtered`
- Optional Odometry: `/wheel/odom`

## Tuning Notes

- Update `config/ekf.yaml` to adjust measurement covariances and sensor bindings.
- Tune `wheel_radius`, `wheel_base`, and encoder tick parameters in `mapping.launch.py` or node parameters.
- Adjust IMU covariance values in `serial_sensor_node.py` to match your sensor quality.
- Calibration of steering encoder zero offset is required for accurate heading.

## Dependencies

- ROS2 Humble
- `pyserial`
- `robot_localization`
- `slam_toolbox`
- `tf2_ros`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`

## Notes

- The serial node reconnects automatically if the Arduino disconnects.
- The odometry node uses a 2D tricycle-like model for front-steer motion.
- This package is designed for live indoor mapping with SLAM Toolbox.
