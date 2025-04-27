# Documentation Updates

## 1. Control Modes: MIT_TORQUE and IMPEDANCE_GRASP

### MIT_TORQUE (0x66)
MIT_TORQUE mode allows direct proportional force control while maintaining position control. Named after the MIT control approach, this mode is useful when:
- Fine force control is needed during manipulation
- The robot needs to interact with delicate objects
- You want direct control over both position and applied force

In this mode, you can set a target position while dynamically adjusting the force/torque limits during movement. This creates a balance between position tracking and force limitation.

### IMPEDANCE_GRASP (0x77)
IMPEDANCE_GRASP mode is optimized for grasping objects safely and effectively. This mode:
- Provides a soft, compliant grasp that adapts to object shape
- Automatically detects contact with objects and reduces force
- Prevents damage to both the hand and manipulated objects
- Maintains position control until contact is detected

This is the default and recommended mode for most grasping operations. The impedance value in feedback indicates resistance to movement (lower values indicate higher resistance).

Usage example:
```python
# Safe grasping with impedance control
hand.move_joints(
    th_mcp=30,
    ff_mcp=45,
    mf_mcp=45,
    rf_mcp=45,
    lf_mcp=45,
    control_mode=ControlMode.IMPEDANCE_GRASP
)
```

## 2. ROS Node Documentation

The DexHand ROS node provides seamless integration with ROS1/ROS2 robotic systems. 

### Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `hands` | Hands to control ("left", "right", or both) | ["right"] |
| `mode` | Control mode (see control modes section) | "impedance_grasp" |
| `rate` | Command send rate in Hz | 100.0 |
| `alpha` | Filter coefficient (0.0-1.0, higher = more responsive) | 0.1 |
| `use_broadcast` | Use efficient broadcast commands | false |
| `enable_feedback` | Enable feedback publishing | false |
| `mock` | Use mock hardware for testing | false |

### Topic Configuration

| Option | Description | Default |
|--------|-------------|---------|
| `left_hand_command_topic` | Left hand joint commands | "/left_hand_joint_commands" |
| `left_hand_joint_feedback_topic` | Left hand joint states | "/left_hand_joint_states" |
| `left_touch_feedback_topic` | Left touch sensor data | "/left_touch_sensors" |
| `left_motor_feedback_topic` | Left motor feedback | "/left_motor_feedback" |
| `right_hand_command_topic` | Right hand joint commands | "/right_hand_joint_commands" |
| `right_hand_joint_feedback_topic` | Right hand joint states | "/right_hand_joint_states" |
| `right_touch_feedback_topic` | Right touch sensor data | "/right_touch_sensors" |
| `right_motor_feedback_topic` | Right motor feedback | "/right_motor_feedback" |

### Feedback Data Structure

#### Touch Sensor Arrays
Published as Float64MultiArray with 40 values (8 values for each of the 5 fingers)
```
Data format per finger:
[0] = timestamp (UNIX time in seconds)
[1] = normal_force (Newtons)
[2] = normal_force_delta (raw units)
[3] = tangential_force (Newtons)
[4] = tangential_force_delta (raw units)
[5] = direction (0-359 degrees, fingertip is 0, -1 if invalid)
[6] = proximity (raw units)
[7] = temperature (Celsius)
```

#### Motor Feedback Arrays
Published as Float64MultiArray with 84 values (7 values for each of the 12 motors)
```
Data format per motor:
[0] = timestamp (UNIX time in seconds)
[1] = angle (degrees)
[2] = encoder_position (raw units, 0-4095)
[3] = current (mA)
[4] = velocity (rpm)
[5] = error_code (0 if no error)
[6] = impedance (lower values = higher resistance to movement)
```

## 3. GUI Slider Example

The `dexhand_gui.py` example provides an interactive GUI for controlling the DexHand using sliders. This application:

- Creates a Qt-based interface with sliders for each joint
- Allows real-time control of both left and right hands
- Provides immediate visual feedback on joint positions
- Includes buttons for resetting joint positions
- Allows copying position dictionaries for use in Python code

The GUI maps each slider's value to the corresponding joint angle and sends commands to the hand in real time. This is useful for:
- Testing hand functionality
- Creating specific hand poses
- Learning the joint range and behavior
- Developing and testing grasp configurations

## 4. Command Types: Broadcast vs Global

### Broadcast Commands (CAN ID 0x100)
Broadcast commands send a single CAN frame that affects all motors simultaneously:
- Used primarily for motion control across all joints
- More efficient than sending individual commands to each board
- Includes position, speed, and current parameters for all 12 motors
- Supports clear_error and request_feedback flags
- Used with `use_broadcast=True` parameter in move_joints()

### Global Commands
Global commands affect all control boards using a special command type:
- Used for system-wide administrative operations
- Includes functions like clearing errors, rebooting, or changing sensor modes
- Defined by GlobalFunctionCode enum (CAN_CANFD_SWITCH, FACTORY_RESET, etc.)
- More suitable for system management than motion control
- Used with `send_global_command()` method

## 5. Additional Joint Movement Parameters

The move_joints() method supports several parameters beyond joint angles:

### Control Mode
Sets the motor control strategy (see Control Modes section)
```python
control_mode=ControlMode.IMPEDANCE_GRASP  # Default
```

### Speeds
Controls motor velocity in RPM:
```python
# Single value for all motors
speeds=15000  # Default

# Individual speeds for each motor
speeds=[10000, 12000, 15000, ...]  # Must have 12 values
```

### Currents
Sets motor current limits in mA:
```python
# Single value for all motors
currents=20  # Default in mA

# Individual currents for each motor
currents=[20, 30, 40, ...]  # Must have 12 values
```

### Broadcast Mode
Enables more efficient command transmission:
```python
use_broadcast=True  # Default is False
```

### Feedback Request
Controls whether to request feedback with the broadcast command:
```python
request_feedback=True  # Default
```