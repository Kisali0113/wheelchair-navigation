# Gate Controller Node & Speaker Node

## Overview

This package contains two ROS2 Humble nodes for robotic platform control:

1. **`gate_controller`**: Manages chair opening and closing operations with permission-based workflow.
2. **`speaker_node`**: Handles voice interactions, announcements, and AI-powered conversations using Google APIs.

---

## Gate Controller Node

### Overview

The `gate_controller` implements a permission-based workflow that requires approval from speaker or caregiver modules before executing gate state changes.

### Topics

#### Published Topics

#### 1. **chair_status**
- **Type:** `std_msgs/String`
- **Description:** Publishes the current state of the chair (gate).
- **Messages:**
  - `chair_closed` - Chair is in closed state (default on startup)
  - `chair_open` - Chair is in open state

#### 2. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Sends requests and emergency messages to the speaker module.
- **Messages:**
  - `requestopen` - Request permission from speaker to open the gate
  - `requestclose` - Request permission from speaker to close the gate
  - `emergency` - Relay emergency message received from maincontrolling

#### 3. **caregiver_control**
- **Type:** `std_msgs/String`
- **Description:** Sends requests to the caregiver module for gate operation approval.
- **Messages:**
  - `gateopenermission` - Request permission from caregiver to open the gate
  - `gateclosermission` - Request permission from caregiver to close the gate

#### 4. **maincontrolling**
- **Type:** `std_msgs/String`
- **Description:** Sends gate control commands to the main controller after permissions are granted.
- **Messages:**
  - `gate_open` - Execute gate open operation
  - `gate_close` - Execute gate close operation

#### Subscribed Topics

#### 1. **robot_command**
- **Type:** `std_msgs/String`
- **Description:** Receives navigation commands from the robot platform.
- **Messages:**
  - `arrived_at_chair` - Robot arrived at chair location → triggers open workflow (if gate closed)
  - `arrived_at_bed` - Robot arrived at bed (ignored by gate controller)
  - `arrived_at_washroom` - Robot arrived at washroom (ignored by gate controller)

#### 2. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Receives permission responses from the speaker module.
- **Messages:**
  - `granted_speaker` - Speaker grants permission for gate operation

#### 3. **caregiver_control**
- **Type:** `std_msgs/String`
- **Description:** Receives permission responses from the caregiver module.
- **Messages:**
  - `granted_caregiver` - Caregiver grants permission for gate operation

#### 4. **maincontrolling**
- **Type:** `std_msgs/String`
- **Description:** Receives status feedback and additional commands from main controller.
- **Messages:**
  - `gateopendsuccess` - Gate open operation completed successfully → updates chair_status to `chair_open`
  - `gatecloserequest` - Main controller requests gate closure → triggers close workflow
  - `gateclosesuccess` - Gate close operation completed successfully → updates chair_status to `chair_closed`
  - `granted_speaker` - Speaker grants permission via main controller
  - `emergency` - Emergency signal from main controller → relayed to speaker_control

### Operating Flows

[Same as before, truncated for brevity]

---

## Speaker Node

### Overview

The `speaker_node` handles voice-based interactions for the robotic platform, including announcements, user confirmations, and AI-powered conversations using Google Cloud APIs.

### Dependencies

- `google-cloud-speech` - For speech-to-text
- `google-cloud-texttospeech` - For text-to-speech
- `google-generativeai` - For Gemini AI interactions
- `pyaudio` - For audio playback
- `speechrecognition` - For microphone input

### Setup Requirements

1. **Google Credentials**: Place `google_credentials.json` (service account key) in the `my_robot_pkg` directory.
2. **Gemini API Key**: Configured in code (update if needed).
3. **Audio Hardware**: Microphone for input, speakers for output.

### Topics

#### Published Topics

#### 1. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Publishes permission grants back to the system.
- **Messages:**
  - `granted_speaker` - User confirmed permission via voice

#### Subscribed Topics

#### 1. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Receives requests for voice interactions.
- **Messages:**
  - `requestopen` - Announce arrival and request confirmation for opening
  - `requestclose` - Announce closing and request confirmation for closing

#### 2. **maincontrolling**
- **Type:** `std_msgs/String`
- **Description:** Receives activation commands for AI mode.
- **Messages:**
  - `speakeractive` - Activate AI conversation mode (STT → Gemini → TTS loop)

### Operating Flows

#### Flow 1: Gate Open Confirmation

```
1. Gate Controller publishes:
   - speaker_control: "requestopen"
   ↓
2. Speaker Node:
   - Announces: "Wheelchair has arrived. Be careful, chair is opening. Can I proceed?"
   - Listens for user response
   ↓
3. User says: "yes" or "yes proceed"
   ↓
4. Speaker Node publishes:
   - speaker_control: "granted_speaker"
```

#### Flow 2: Gate Close Confirmation

```
1. Gate Controller publishes:
   - speaker_control: "requestclose"
   ↓
2. Speaker Node:
   - Announces: "Chair is closing. Can I proceed?"
   - Listens for user response
   ↓
3. User says: "yes" or "yes proceed"
   ↓
4. Speaker Node publishes:
   - speaker_control: "granted_speaker"
```

#### Flow 3: Emergency Sound Track

```
1. Gate Controller or Main Controller publishes:
   - speaker_control: "emergency"
   ↓
2. Speaker Node:
   - Plays pre-recorded audio file "elevenlabs.wav" from package directory
   ↓
3. Emergency sound track plays through speakers
```

### Usage

#### Run Speaker Node

```bash
source install/setup.bash
ros2 run my_robot_pkg speaker_node
```

#### Test Commands

```bash
# Trigger open confirmation
ros2 topic pub /speaker_control std_msgs/String "{data: 'requestopen'}" -1

# Activate AI mode
ros2 topic pub /maincontrolling std_msgs/String "{data: 'speakeractive'}" -1
```

### Implementation Notes

- Uses threading for non-blocking speech operations
- Speech recognition has 5-second timeout and 10-second phrase limit
- Gemini responses are spoken immediately after generation
- Extensible structure for additional voice commands and topics

---

## General Usage

### Build

```bash
cd ~/fyp_ws
colcon build --packages-select my_robot_pkg
```

### Run Both Nodes

```bash
source install/setup.bash
ros2 run my_robot_pkg gate_controller &
ros2 run my_robot_pkg speaker_node &
```

### Monitor

```bash
# Watch topics
ros2 topic echo /chair_status
ros2 topic echo /speaker_control
ros2 topic echo /odom
```

---

## Odometry Localizer Node

### Overview

The `odom_localizer` node reads encoder data from an Arduino over serial and computes robot odometry using a front-wheel steering bicycle model.

### Topics

#### Published Topics

#### 1. **odom**
- **Type:** `nav_msgs/Odometry`
- **Description:** Publishes estimated robot pose and velocity.

### Serial Input

The node reads lines from the Arduino serial port. Supported formats include:
- `0.12,15`
- `0.12 15`
- `speed=0.12,angle=15`
- `rpm=1200,angle=15` (wheel rotational speed can be converted using tire diameter)
- `rps=20,angle=15`

Where the values are:
- `speed`: front wheel linear speed (m/s)
- `angle`: front wheel steering angle (degrees or radians)
- `rpm`: front wheel rotational speed, converted with tire diameter 220 mm
- `rps`: front wheel rotations per second

### Parameters

- `serial_port` - Serial device path (default: `/dev/ttyUSB0`)
- `baud_rate` - Serial baud rate (default: `115200`)
- `wheelbase_mm` - Distance between front and rear wheel (default: `1032.33`)
- `track_width_mm` - Robot track width or rear axle width (default: `746.2`)
- `tire_diameter_mm` - Front wheel tire diameter (default: `220.0`)
- `frame_id` - Odometry frame (default: `odom`)
- `child_frame_id` - Robot base frame (default: `base_link`)

### Run

```bash
source install/setup.bash
ros2 run my_robot_pkg odom_localizer
```

---

## Human Position Node

### Overview

The `human_position_node` activates when it receives `personsearching` on the `robotcommand` topic. It uses a RealSense depth camera, YOLOv8 human detection, and DBSCAN clustering to publish the detected person's 3D position.

### Topics

#### Subscribed Topics

#### 1. **robotcommand**
- **Type:** `std_msgs/String`
- **Description:** Receives control commands.
- **Messages:**
  - `personsearching` - Activate human detection and publish the person's camera-space position.

#### Published Topics

#### 1. **person_position**
- **Type:** `geometry_msgs/PointStamped`
- **Description:** Publishes the detected person's position in the camera frame.
- **Fields:**
  - `point.x` - X coordinate in meters
  - `point.y` - Y coordinate in meters
  - `point.z` - Z coordinate in meters

### Run

```bash
source install/setup.bash
ros2 run my_robot_pkg human_position_node
```

---

## Person Tracker Node

### Overview

The `person_tracker_node` activates when it receives `followperson` on the `microcontrolling` topic. It uses RealSense depth camera, YOLOv8 human detection, DeepSort tracking, and color histogram re-identification to track the closest person and publish their position.

### Topics

#### Subscribed Topics

#### 1. **microcontrolling**
- **Type:** `std_msgs/String`
- **Description:** Receives control commands.
- **Messages:**
  - `followperson` - Activate person tracking and publish position updates.

#### Published Topics

#### 1. **person_position**
- **Type:** `geometry_msgs/PointStamped`
- **Description:** Publishes the tracked person's position in camera frame.
- **Fields:**
  - `point.x` - X coordinate in meters
  - `point.y` - Y coordinate in meters
  - `point.z` - Z coordinate (distance) in meters

### Run

```bash
source install/setup.bash
ros2 run my_robot_pkg person_tracker_node
```

---

## Future Enhancements

- Add timeout handling for voice responses
- Implement wake word detection for AI mode
- Support multiple languages for STT/TTS
- Add conversation history for Gemini context
- Integrate with additional ROS2 topics for expanded functionality

### Published Topics

#### 1. **chair_status**
- **Type:** `std_msgs/String`
- **Description:** Publishes the current state of the chair (gate).
- **Messages:**
  - `chair_closed` - Chair is in closed state (default on startup)
  - `chair_open` - Chair is in open state

#### 2. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Sends requests and emergency messages to the speaker module.
- **Messages:**
  - `requestopen` - Request permission from speaker to open the gate
  - `requestclose` - Request permission from speaker to close the gate
  - `emergency` - Relay emergency message received from maincontrolling

#### 3. **caregiver_control**
- **Type:** `std_msgs/String`
- **Description:** Sends requests to the caregiver module for gate operation approval.
- **Messages:**
  - `gateopenermission` - Request permission from caregiver to open the gate
  - `gateclosermission` - Request permission from caregiver to close the gate

#### 4. **maincontrolling**
- **Type:** `std_msgs/String`
- **Description:** Sends gate control commands to the main controller after permissions are granted.
- **Messages:**
  - `gate_open` - Execute gate open operation
  - `gate_close` - Execute gate close operation

---

### Subscribed Topics

#### 1. **robot_command**
- **Type:** `std_msgs/String`
- **Description:** Receives navigation commands from the robot platform.
- **Messages:**
  - `arrived_at_chair` - Robot arrived at chair location → triggers open workflow (if gate closed)
  - `arrived_at_bed` - Robot arrived at bed (ignored by gate controller)
  - `arrived_at_washroom` - Robot arrived at washroom (ignored by gate controller)

#### 2. **speaker_control**
- **Type:** `std_msgs/String`
- **Description:** Receives permission responses from the speaker module.
- **Messages:**
  - `granted_speaker` - Speaker grants permission for gate operation

#### 3. **caregiver_control**
- **Type:** `std_msgs/String`
- **Description:** Receives permission responses from the caregiver module.
- **Messages:**
  - `granted_caregiver` - Caregiver grants permission for gate operation

#### 4. **maincontrolling**
- **Type:** `std_msgs/String`
- **Description:** Receives status feedback and additional commands from main controller.
- **Messages:**
  - `gateopendsuccess` - Gate open operation completed successfully → updates chair_status to `chair_open`
  - `gatecloserequest` - Main controller requests gate closure → triggers close workflow
  - `gateclosesuccess` - Gate close operation completed successfully → updates chair_status to `chair_closed`
  - `granted_speaker` - Speaker grants permission via main controller
  - `emergency` - Emergency signal from main controller → relayed to speaker_control

---

## Operating Flows

### Flow 1: Open Gate

```
1. Robot Command: arrived_at_chair
   ↓
2. Gate Controller publishes:
   - speaker_control: "requestopen"
   - caregiver_control: "gateopenermission"
   ↓
3. Wait for permission grants (state flags: speaker_granted, caregiver_granted)
   ↓
4. Permission granted when:
   - caregiver_granted = True, OR
   - speaker_granted = True AND caregiver_granted = True
   ↓
5. Gate Controller publishes:
   - maincontrolling: "gate_open"
   ↓
6. Main Controller executes open and publishes:
   - maincontrolling: "gateopendsuccess"
   ↓
7. Gate Controller updates:
   - gate_state = "open"
   - chair_status: "chair_open"
```

### Flow 2: Close Gate

```
1. Main Controller publishes:
   - maincontrolling: "gatecloserequest"
   ↓
2. Gate Controller publishes:
   - speaker_control: "requestclose"
   - caregiver_control: "gateclosermission"
   ↓
3. Wait for permission grants (state flags: speaker_granted, caregiver_granted)
   ↓
4. Permission granted when:
   - caregiver_granted = True, OR
   - speaker_granted = True AND caregiver_granted = True
   ↓
5. Gate Controller publishes:
   - maincontrolling: "gate_close"
   ↓
6. Main Controller executes close and publishes:
   - maincontrolling: "gateclosesuccess"
   ↓
7. Gate Controller updates:
   - gate_state = "closed"
   - chair_status: "chair_closed"
```

### Flow 3: Emergency Handling

```
1. Main Controller publishes:
   - maincontrolling: "emergency"
   ↓
2. Gate Controller immediately publishes:
   - speaker_control: "emergency"
   ↓
3. Speaker module handles emergency (e.g., alert user, trigger safety protocols)
```

### Flow 4: Speaker Permission via Main Controller

```
1. Main Controller publishes:
   - maincontrolling: "granted_speaker"
   ↓
2. Gate Controller:
   - Sets speaker_granted = True
   - Evaluates permission conditions
   - If conditions met and waiting for open/close, publishes gate_open/gate_close
```

---

## State Machine

### Node States

| State Variable | Type | Values | Purpose |
|---|---|---|---|
| `gate_state` | str | `'closed'`, `'open'` | Tracks current physical gate position |
| `speaker_granted` | bool | `True`, `False` | Permission status from speaker module |
| `caregiver_granted` | bool | `True`, `False` | Permission status from caregiver module |
| `waiting_for_open` | bool | `True`, `False` | Flag for pending open operation |
| `waiting_for_close` | bool | `True`, `False` | Flag for pending close operation |

### Initial State

- `gate_state = 'closed'` (chair is closed by default)
- `speaker_granted = False`
- `caregiver_granted = False`
- `waiting_for_open = False`
- `waiting_for_close = False`
- First message published: `chair_status: 'chair_closed'`

---

## Permission Logic

**Gate operations are authorized when:**

```
(caregiver_granted == True) OR (speaker_granted == True AND caregiver_granted == True)
```

This means:
- ✅ Caregiver permission alone is sufficient
- ✅ Both speaker and caregiver permissions satisfy the condition
- ❌ Speaker permission alone is NOT sufficient

---

## Usage

### Build

```bash
cd ~/fyp_ws
colcon build --packages-select my_robot_pkg
```

### Run

```bash
source install/setup.bash
ros2 run my_robot_pkg gate_controller
```

### Test (Publish Test Commands)

```bash
# Open gate workflow
ros2 topic pub /robot_command std_msgs/String "{data: 'arrived_at_chair'}" -1

# Grant speaker permission
ros2 topic pub /speaker_control std_msgs/String "{data: 'granted_speaker'}" -1

# Grant caregiver permission
ros2 topic pub /caregiver_control std_msgs/String "{data: 'granted_caregiver'}" -1

# Simulate successful open
ros2 topic pub /maincontrolling std_msgs/String "{data: 'gateopendsuccess'}" -1

# Request close
ros2 topic pub /maincontrolling std_msgs/String "{data: 'gatecloserequest'}" -1

# Emergency signal
ros2 topic pub /maincontrolling std_msgs/String "{data: 'emergency'}" -1
```

### Monitor Output

```bash
# Watch chair status updates
ros2 topic echo /chair_status

# Monitor all published commands
ros2 topic echo /speaker_control
ros2 topic echo /caregiver_control
ros2 topic echo /maincontrolling
```

---

## Implementation Details

### Key Methods

| Method | Purpose |
|---|---|
| `publish_chair_status(status)` | Publish chair open/closed status |
| `request_open()` | Initiate open permission workflow |
| `request_close()` | Initiate close permission workflow |
| `evaluate_gate_permission()` | Check if permission conditions met and execute gate commands |
| `robot_command_callback(msg)` | Handle navigation commands |
| `speaker_control_callback(msg)` | Handle speaker permission grants |
| `caregiver_control_callback(msg)` | Handle caregiver permission grants |
| `maincontrolling_callback(msg)` | Handle status feedback and additional commands |

### Logging

The node logs all significant events to help with debugging:
- Topic messages received and published
- Permission state changes
- Gate state transitions
- Errors and warnings

View logs:

```bash
ros2 run my_robot_pkg gate_controller 2>&1 | tee gate_controller.log
```

---

## Future Enhancements

- Add timeout handling for permission requests (auto-cancel if no response)
- Implement persistent state storage (save gate state to file)
- Add parameter configuration for permission requirements
- Support custom message types instead of String
- Add diagnostics publisher for health monitoring

---

## Package Dependencies

- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 message types

---

# Autonomous Navigation Nodes

## Overview

This package includes a complete autonomous navigation stack for Ackermann steering vehicles:

1. **`goal_sender_node`**: Accepts goal coordinates and sends them to Nav2
2. **`path_receiver_node`**: Subscribes to planned paths from Nav2 planner
3. **`pure_pursuit_controller_node`**: Implements pure pursuit control for path following
4. **`pwm_conversion_node`**: Converts steering commands to PWM signals for motors/servos

## Navigation System Architecture

```
Goal Input (x,y)
    ↓
Goal Sender Node
    ↓
Nav2 Planner (Smac Hybrid A*)
    ↓
Path Receiver Node
    ↓
Pure Pursuit Controller
    ↓
Ackermann Drive Commands
    ↓
PWM Conversion Node
    ↓
Motor/Servo PWM Signals
    ↓
Arduino Mega
```

## Goal Sender Node

### Overview
Converts simple x,y goal coordinates into PoseStamped messages for Nav2 navigation.

### Topics
- **Subscribes:** 
  - `/goal_xy` (Float32MultiArray) - x,y goal coordinates
  - `/goal_pose` (PoseStamped) - PoseStamped goals from other nodes
  - `/move_base_simple/goal` (PoseStamped) - RViz goal tool
- **Publishes:** `/goal_pose` (PoseStamped), Nav2 action server

### Usage
```bash
# Send goal via command line
ros2 topic pub /goal_xy std_msgs/Float32MultiArray "data: [2.0, 3.0, 0.0]"

# Send goal via RViz
# 1. Open RViz
# 2. Add "2D Goal Pose" tool
# 3. Click and drag to set goal position and orientation
# 4. Goal will be automatically sent to Nav2
```

## Path Receiver Node

### Overview
Receives and stores planned paths from Nav2 for controller use.

### Topics
- **Subscribes:** `/plan` (Path)
- **Publishes:** `/current_path` (Path)

### Features
- Path validation and age checking
- Waypoint extraction for lookahead calculations
- Closest waypoint finding

## Pure Pursuit Controller Node

### Overview
Implements pure pursuit algorithm for Ackermann steering control.

**Pure Pursuit Formula:**
```
delta = atan((2 * L * sin(alpha)) / Ld)
```

Where:
- `delta`: Steering angle
- `L`: Wheelbase (0.42 m)
- `alpha`: Heading error to lookahead point
- `Ld`: Lookahead distance

### Topics
- **Subscribes:** `/odometry/filtered` (Odometry), `/plan` (Path)
- **Publishes:** `/ackermann_cmd` (AckermannDriveStamped), `/lookahead_point` (PointStamped)

### Parameters
- `wheelbase`: 0.42 m
- `lookahead_distance`: 1.0 m
- `target_speed`: 0.5 m/s
- `max_steering_angle`: 0.5236 rad (~30°)

## PWM Conversion Node

### Overview
Converts Ackermann drive commands into PWM values for motor and servo control.

### PWM Ranges
- **Steering Servo:** 1000-2000 μs (neutral: 1500 μs)
- **Drive Motor:** 0-255 (stop: 127)

### Topics
- **Subscribes:** `/ackermann_cmd` (AckermannDriveStamped)
- **Publishes:** `/motor_pwm` (UInt16MultiArray)

## Launch Navigation System

```bash
# Launch all navigation nodes
ros2 launch my_robot_pkg navigation.launch.py \
    wheelbase:=0.42 \
    lookahead_distance:=1.0 \
    target_speed:=0.5 \
    max_steering_angle:=0.5236
```

## Individual Node Launch

```bash
# Goal sender
ros2 run my_robot_pkg goal_sender_node

# Path receiver
ros2 run my_robot_pkg path_receiver_node

# Pure pursuit controller
ros2 run my_robot_pkg pure_pursuit_controller_node

# PWM conversion
ros2 run my_robot_pkg pwm_conversion_node
```

## Testing Navigation

### Send Test Goal
```bash
# Using the test script
python3 navigation_test.py 2.0 3.0 0.0

# Or directly with ros2 topic
ros2 topic pub /goal_xy std_msgs/Float32MultiArray "data: [2.0, 3.0, 0.0]"
# Or using RViz 2D Goal Pose tool (recommended)
# 1. ros2 run rviz2 rviz2
# 2. Add displays: Map, RobotModel, Path
# 3. Add tool: 2D Goal Pose
# 4. Click and drag in map to set goal```

### Monitor System
```bash
# Check active topics
ros2 topic list -t

# Monitor PWM output
ros2 topic echo /motor_pwm

# Monitor controller commands
ros2 topic echo /ackermann_cmd

# View lookahead point
ros2 topic echo /lookahead_point
```

## RViz Integration

The goal sender node supports RViz for interactive goal setting:

### Setup RViz for Navigation
```bash
# Launch RViz with navigation config
ros2 run rviz2 rviz2 -d src/my_robot_pkg/config/navigation.rviz

# Or manually configure:
# 1. Fixed Frame: map
# 2. Add Map display (topic: /map)
# 3. Add RobotModel display
# 4. Add Odometry display (topic: /odometry/filtered)
# 5. Add Path display (topic: /plan for global plan)
# 6. Add Path display (topic: /current_path for current path)
# 7. Add Pose display (topic: /goal_pose for goals)
# 8. Add Position display (topic: /lookahead_point for debugging)
# 9. Add TF display
# 10. Add 2D Goal Pose tool
```

### Using RViz Goal Tool
1. **Select 2D Goal Pose Tool** from the toolbar
2. **Click and drag** on the map to set:
   - Goal position (where you click)
   - Goal orientation (direction you drag)
3. **Goal is automatically sent** to Nav2 navigation stack
4. **Monitor progress** in RViz displays

### RViz Topics Used
- **`/move_base_simple/goal`** - RViz publishes goals here
- **`/goal_pose`** - Republished goals for visualization
- **`/plan`** - Global path from Nav2 planner
- **`/current_path`** - Current path from path receiver
- **`/lookahead_point`** - Controller lookahead point

## Integration with Nav2

The navigation nodes are designed to work with Nav2 stack:

1. **AMCL** provides localization (`/odometry/filtered`)
2. **Smac Hybrid A*** planner generates paths (`/plan`)
3. **Navigation nodes** handle path following and actuation

### Required Nav2 Configuration
- Global costmap with static map
- Smac Hybrid A* planner
- Regulated Pure Pursuit controller (can be disabled if using our nodes)
- Proper robot footprint and inflation settings

## Hardware Integration

### Arduino Mega Setup
```cpp
// PWM channels
#define STEERING_SERVO_PIN 9   // PWM channel 0
#define MOTOR_PIN 10          // PWM channel 1

// PWM ranges (configure in pwm_conversion_node)
#define STEERING_MIN 1000
#define STEERING_MAX 2000
#define STEERING_NEUTRAL 1500
#define MOTOR_MIN 0
#define MOTOR_MAX 255
#define MOTOR_STOP 127
```

### Serial Communication
The PWM values are published as `UInt16MultiArray` for rosserial compatibility.

## Troubleshooting

### No Path Received
```bash
# Check if Nav2 is running
ros2 node list | grep nav2

# Check if planner is publishing
ros2 topic echo /plan
```

### Controller Not Publishing
```bash
# Check odometry
ros2 topic echo /odometry/filtered

# Check path validity
ros2 topic echo /current_path
```

### PWM Values Incorrect
```bash
# Check Ackermann commands
ros2 topic echo /ackermann_cmd

# Verify PWM conversion parameters
ros2 param get /pwm_conversion steering_pwm_neutral
```

## Performance Tuning

### Controller Parameters
- **lookahead_distance**: Increase for smoother turns, decrease for tighter following
- **target_speed**: Adjust based on vehicle capabilities
- **speed_reduction_ratio**: Controls speed reduction in turns

### PWM Calibration
- **steering_pwm_neutral**: Calibrate for straight driving
- **steering_angle_max**: Match physical steering limits
- **motor_pwm_stop**: Calibrate for zero velocity

## Dependencies

### ROS2 Packages
- `nav2_bringup` - Navigation stack
- `ackermann_msgs` - Ackermann drive messages
- `nav2_msgs` - Navigation action messages
- `tf2_geometry_msgs` - TF transformations

### Python Packages
- `numpy` - Mathematical operations
- `transforms3d` - 3D transformations (if needed)

---

*Navigation System Documentation - Version 1.0*

