#!/usr/bin/env python3
from ros_compat import ROSNode
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import argparse
import numpy as np
from typing import Dict, List, Optional
from enum import Enum
import time
import yaml
import os.path

from pyzlg_dexhand.dexhand_interface import (
    DexHandBase,
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
)
from pyzlg_dexhand.zcan_wrapper import MockZCANWrapper

from filters import DampedVelocityKalmanFilter


class HardwareMapping(Enum):
    """Mapping between URDF and hardware joints"""

    th_dip = ("th_dip", ["f_joint1_3", "f_joint1_4"])
    th_mcp = ("th_mcp", ["f_joint1_2"])
    th_rot = ("th_rot", ["f_joint1_1"])
    ff_spr = ("ff_spr", ["f_joint2_1", "f_joint4_1", "f_joint5_1"])
    ff_dip = ("ff_dip", ["f_joint2_3", "f_joint2_4"])
    ff_mcp = ("ff_mcp", ["f_joint2_2"])
    mf_dip = ("mf_dip", ["f_joint3_3", "f_joint3_4"])
    mf_mcp = ("mf_mcp", ["f_joint3_2"])
    rf_dip = ("rf_dip", ["f_joint4_3", "f_joint4_4"])
    rc_mcp = ("rf_mcp", ["f_joint4_2"])
    lf_dip = ("lf_dip", ["f_joint5_3", "f_joint5_4"])
    lf_mcp = ("lf_mcp", ["f_joint5_2"])


class JointMapping:
    """Maps between URDF joints and hardware joints"""

    def __init__(self, prefix: str = "l"):
        """Initialize joint mapping"""
        self.prefix = prefix

        # Create mapping from URDF joint names to hardware joints
        self.urdf_to_hw = {}
        for hw_mapping in HardwareMapping:
            dex_joint, urdf_joints = hw_mapping.value
            for urdf_joint in urdf_joints:
                full_name = f"{prefix}_{urdf_joint}"
                self.urdf_to_hw[full_name] = dex_joint

        # Get all possible URDF joint names
        self.joint_names = sorted(list(self.urdf_to_hw.keys()))

    def map_command(self, joint_values: Dict[str, float]) -> Dict[str, float]:
        """Map URDF joint values to hardware commands"""
        # Group joint values by hardware joint
        hw_joint_values = {dex_joint: [] for dex_joint in DexHandBase.joint_names}

        for name, value in joint_values.items():
            if name in self.urdf_to_hw:
                dex_joint = self.urdf_to_hw[name]
                hw_joint_values[dex_joint].append(value)

        # Average values for each hardware joint
        command = {}
        for dex_joint in DexHandBase.joint_names:
            values = hw_joint_values[dex_joint]
            if values:
                command[dex_joint] = float(np.rad2deg(sum(values) / len(values)))

        return command

    def map_feedback(self, hardware_values: Dict[str, float]) -> Dict[str, float]:
        """Map hardware joint values to URDF joint values"""
        joint_state_dict = {}
        for urdf_joint in self.joint_names:
            dex_joint = self.urdf_to_hw.get(urdf_joint)
            if dex_joint in hardware_values:
                joint_state_dict[urdf_joint] = float(np.deg2rad(hardware_values[dex_joint]))
        return joint_state_dict


class DexHandNode(ROSNode):
    """ROS2 Node for controlling one or both DexHands"""

    def __init__(self, config: dict):
        super().__init__("dexhand")


        # Get configuration values
        hands = config.get("hands", ["right"])
        control_mode = config.get("mode", "impedance_grasp")
        send_rate = config.get("rate", 100.0)
        filter_alpha = config.get("alpha", 0.1)
        self.left_hand_command_topic = config.get("left_hand_command_topic", "/left_hand_joint_commands")
        self.right_hand_command_topic = config.get("right_hand_command_topic", "/right_hand_joint_commands")
        self.left_hand_joint_feedback_topic = config.get("left_hand_joint_feedback_topic", "/left_hand_joint_states")
        self.left_touch_feedback_topic = config.get("left_touch_feedback_topic", "/left_touch_sensors")
        self.right_hand_joint_feedback_topic = config.get("right_hand_joint_feedback_topic", "/left_hand_joint_states")
        self.right_touch_feedback_topic = config.get("right_touch_feedback_topic", "/right_touch_sensors")

        self.is_mock = config.get("mock", False)

        # Add feedback configuration
        self.enable_feedback = config.get("enable_feedback", False)
        self.feedback_dt = 1.0 / send_rate  # Time step for Kalman filter

        # Initialize shared ZCAN
        self.zcan = ZCANWrapper() if not self.is_mock else MockZCANWrapper()
        if not self.zcan.open():
            raise RuntimeError("Failed to open ZCAN device")

        # Set up control mode
        self.control_mode_map = {
            "zero_torque": ControlMode.ZERO_TORQUE,
            "current": ControlMode.CURRENT,
            "speed": ControlMode.SPEED,
            "hall_position": ControlMode.HALL_POSITION,
            "cascaded_pid": ControlMode.CASCADED_PID,
            "protect_hall_position": ControlMode.PROTECT_HALL_POSITION,
            "mit_torque":ControlMode.MIT_TORQUE,
            "impedance_grasp":ControlMode.IMPEDANCE_GRASP
        }
        self.control_mode = self.control_mode_map.get(
            control_mode, ControlMode.IMPEDANCE_GRASP
        )
        self.filter_alpha = filter_alpha

        # Initialize hands with shared ZCAN
        self.hands = {}
        self.joint_mappings = {}
        self.last_commands = {}

        # Initialize Kalman filters for each joint when feedback is enabled
        self.kalman_filters = {}
        self.last_joint_positions = {}
        self.fingertip_mapping = {
            "th": 0,  # Thumb
            "ff": 1,  # Index
            "mf": 2,  # Middle
            "rf": 3,  # Ring
            "lf": 4,  # Pinky
        }

        for hand in hands:
            # Initialize hand with shared ZCAN
            hand_class = LeftDexHand if hand == "left" else RightDexHand
            self.hands[hand] = hand_class(self.zcan)
            if not self.hands[hand].init():
                raise RuntimeError(f"Failed to initialize {hand} hand")

            # Initialize joint mapping
            self.joint_mappings[hand] = JointMapping("l" if hand == "left" else "r")

            # Initialize last command
            self.last_commands[hand] = {}

            # Initialize Kalman filters and state tracking for each joint
            if self.enable_feedback:
                self.kalman_filters[hand] = {}
                self.last_joint_positions[hand] = {}

                for joint_name in DexHandBase.joint_names:
                    # Parameters for Kalman filter: dt, process_noise_var, measurement_noise_var, damping
                    self.kalman_filters[hand][joint_name] = DampedVelocityKalmanFilter(
                        dt=self.feedback_dt,
                        process_noise_var=100.0,
                        measurement_noise_var=0.1,
                        damping=0.9,
                    )
                    self.last_joint_positions[hand][joint_name] = 0.0

        # Initialize command subscriber with configurable topic
        self.create_subscription(JointState, self.left_hand_command_topic, self.left_hand_command_callback)
        self.create_subscription(JointState, self.right_hand_command_topic, self.right_hand_command_callback)

        # Initialize reset service
        self.create_service(Trigger, "reset_hands", self.reset_callback)

        # Initialize publishers for feedback
        if self.enable_feedback:
            self.left_hand_joint_state_pub = self.create_publisher(
                JointState, self.left_hand_joint_feedback_topic, 10
            )
            self.right_hand_joint_state_pub = self.create_publisher(
                JointState, self.right_hand_joint_feedback_topic, 10
            )
            self.left_touch_sensor_pub = self.create_publisher(
                Float32MultiArray, self.left_touch_feedback_topic, 10
            )
            self.right_touch_sensor_pub = self.create_publisher(
                Float32MultiArray, self.right_touch_feedback_topic, 10
            )


        # Set up command sending timer
        period = 1.0 / send_rate
        self.timer = self.create_timer(period, self.send_commands)

        self.logger.info(
            f"DexHand node initialized:\n"
            f'  Hands: {", ".join(hands)}\n'
            f"  Control mode: {control_mode}\n"
            f"  Send rate: {send_rate} Hz\n"
            f"  Filter alpha: {filter_alpha}\n"
            f"  Feedback enabled: {self.enable_feedback}\n"
        )

    def right_hand_command_callback(self, msg: JointState):
        """Handle incoming joint commands"""
        try:
            # Create dictionary of joint values
            joint_values = {}
            for name, pos in zip(msg.name, msg.position):
                joint_values[name] = pos

            # Process command for each hand
            for hand, mapping in self.joint_mappings.items():
                # Map to hardware joints
                command = mapping.map_command(joint_values)

                # Apply low-pass filter
                if not self.last_commands[hand]:
                    # First command, no filtering
                    self.last_commands[hand] = command
                else:
                    for joint, value in command.items():
                        if joint not in self.last_commands[hand]:
                            self.last_commands[hand][joint] = value
                        else:
                            self.last_commands[hand][joint] = (
                                1 - self.filter_alpha
                            ) * self.last_commands[hand][
                                joint
                            ] + self.filter_alpha * value
        except Exception as e:
            self.logger.error(f"Error in command callback: {str(e)}")

    def left_hand_command_callback(self, msg: JointState):
        """Handle incoming joint commands"""
        try:
            # Create dictionary of joint values
            joint_values = {}
            for name, pos in zip(msg.name, msg.position):
                joint_values[name] = pos

            # Process command for each hand
            for hand, mapping in self.joint_mappings.items():
                # Map to hardware joints
                command = mapping.map_command(joint_values)

                # Apply low-pass filter
                if not self.last_commands[hand]:
                    # First command, no filtering
                    self.last_commands[hand] = command
                else:
                    for joint, value in command.items():
                        if joint not in self.last_commands[hand]:
                            self.last_commands[hand][joint] = value
                        else:
                            self.last_commands[hand][joint] = (
                                1 - self.filter_alpha
                            ) * self.last_commands[hand][
                                joint
                            ] + self.filter_alpha * value
        except Exception as e:
            self.logger.error(f"Error in command callback: {str(e)}")

    def send_commands(self):
        """Send filtered commands to all hands"""
        try:
            # Send commands to each hand
            for hand, hand_interface in self.hands.items():
                command = self.last_commands.get(hand, {})

                # Use move_joints interface
                hand_interface.move_joints(
                    th_dip=command.get("th_dip"),
                    th_mcp=command.get("th_mcp"),
                    th_rot=command.get("th_rot"),
                    ff_spr=command.get("ff_spr"),
                    ff_dip=command.get("ff_dip"),
                    ff_mcp=command.get("ff_mcp"),
                    mf_dip=command.get("mf_dip"),
                    mf_mcp=command.get("mf_mcp"),
                    rf_dip=command.get("rf_dip"),
                    rf_mcp=command.get("rf_mcp"),
                    lf_dip=command.get("lf_dip"),
                    lf_mcp=command.get("lf_mcp"),
                    control_mode=self.control_mode,
                )

                # Clear errors
                hand_interface.clear_errors(clear_all=True)

                # Get and publish feedback if enabled
                if self.enable_feedback:
                    # print(hand)
                    self.process_and_publish_feedback(hand)

        except Exception as e:
            self.logger.error(f"Error sending commands: {str(e)}")

    def process_and_publish_feedback(self, hand: str):
        """Get feedback from hardware and publish to ROS topics"""
        try:
            if hand == "left":
                # Get feedback from hand
                feedback_left = self.hands[hand].get_feedback()

                # Process joint positions and estimate velocities
                left_pos_dict = {}
                left_vel_dict = {}
                for joint_name, joint_feedback_left in feedback_left.joints.items():
                    # Get position
                    left_position = joint_feedback_left.angle
                    # Update the Kalman filter with new measurement
                    kalman_filter_left = self.kalman_filters[hand][joint_name]
                    if not np.isnan(left_position):
                        kalman_filter_left.step(left_position)
                    left_state = kalman_filter_left.get_current_state()

                    # Extract position (state[0]) and velocity (state[1])
                    left_velocity = left_state[1][0]  # Extract velocity in deg/s

                    # Save position and velocity to dictionaries
                    left_pos_dict[joint_name] = left_position
                    left_vel_dict[joint_name] = left_velocity

                # Convert hardware feedback to URDF joint names
                left_pos_dict_urdf = self.joint_mappings[hand].map_feedback(left_pos_dict)
                left_vel_dict_urdf = self.joint_mappings[hand].map_feedback(left_vel_dict)

                # Construct and publish joint state message
                left_hand_joint_state_msg = JointState()
                left_hand_joint_state_msg.header.stamp = self.get_ros_time().to_msg()
                left_hand_joint_state_msg.name = self.joint_mappings[hand].joint_names
                left_hand_joint_state_msg.position = [left_pos_dict_urdf.get(joint, 0.0) for joint in left_hand_joint_state_msg.name]
                left_hand_joint_state_msg.velocity = [left_vel_dict_urdf.get(joint, 0.0) for joint in left_hand_joint_state_msg.name]
                self.left_hand_joint_state_pub.publish(left_hand_joint_state_msg)

                # Process tactile feedback
                if feedback_left.tactile:
                    left_touch_msg = Float32MultiArray()
                    left_touch_msg.data = [0.0] * 70  # Default to 0 for all fingertips, 5 fingers * 14 sensor data

                    # Map feedback to correct indices (thumb, index, middle, ring, pinky)
                    for finger_name, tactile_data in feedback_left.tactile.items():
                        if finger_name in self.fingertip_mapping:
                            idx = self.fingertip_mapping[finger_name]
                            left_touch_msg.data[14*idx] = tactile_data.timestamp
                            left_touch_msg.data[14*idx+1] = tactile_data.normal_force
                            left_touch_msg.data[14*idx+2] = tactile_data.normal_force_delta
                            left_touch_msg.data[14*idx+3] = tactile_data.tangential_force
                            left_touch_msg.data[14*idx+4] = tactile_data.tangential_force_delta
                            left_touch_msg.data[14*idx+5] = tactile_data.direction
                            left_touch_msg.data[14*idx+6] = tactile_data.proximity
                            left_touch_msg.data[14*idx+7] = tactile_data.temperature
                            left_touch_msg.data[14*idx+8] = tactile_data.encoder1
                            left_touch_msg.data[14*idx+9] = tactile_data.encoder2
                            left_touch_msg.data[14*idx+10] = tactile_data.motor1_error
                            left_touch_msg.data[14*idx+11] = tactile_data.motor2_error
                            left_touch_msg.data[14*idx+12] = tactile_data.impedance1
                            left_touch_msg.data[14*idx+13] = tactile_data.impedance2

                    # Publish tactile data
                    self.left_touch_sensor_pub.publish(left_touch_msg)
                
            elif hand == "right":
                # Get feedback from hand
                feedback_right = self.hands[hand].get_feedback()

                # Process joint positions and estimate velocities
                right_pos_dict = {}
                right_vel_dict = {}
                for joint_name, joint_feedback_right in feedback_right.joints.items():
                    # Get position
                    right_position = joint_feedback_right.angle
                    # Update the Kalman filter with new measurement
                    kalman_filter_right = self.kalman_filters[hand][joint_name]
                    if not np.isnan(right_position):
                        kalman_filter_right.step(right_position)
                    right_state = kalman_filter_right.get_current_state()

                    # Extract position (state[0]) and velocity (state[1])
                    right_velocity = right_state[1][0]  # Extract velocity in deg/s

                    # Save position and velocity to dictionaries
                    right_pos_dict[joint_name] = right_position
                    right_vel_dict[joint_name] = right_velocity

                # Convert hardware feedback to URDF joint names
                right_pos_dict_urdf = self.joint_mappings[hand].map_feedback(right_pos_dict)
                right_vel_dict_urdf = self.joint_mappings[hand].map_feedback(right_vel_dict)

                # Construct and publish joint state message
                right_hand_joint_state_msg = JointState()
                right_hand_joint_state_msg.header.stamp = self.get_ros_time().to_msg()
                right_hand_joint_state_msg.name = self.joint_mappings[hand].joint_names
                right_hand_joint_state_msg.position = [right_pos_dict_urdf.get(joint, 0.0) for joint in right_hand_joint_state_msg.name]
                right_hand_joint_state_msg.velocity = [right_vel_dict_urdf.get(joint, 0.0) for joint in right_hand_joint_state_msg.name]
                self.right_hand_joint_state_pub.publish(right_hand_joint_state_msg)

                # Process tactile feedback
                if feedback_right.tactile:
                    right_touch_msg = Float32MultiArray()
                    right_touch_msg.data = [0.0] * 70  # Default to 0 for all fingertips, 5 fingers * 14 sensor data

                    # Map feedback to correct indices (thumb, index, middle, ring, pinky)
                    for finger_name, tactile_data in feedback_right.tactile.items():
                        if finger_name in self.fingertip_mapping:
                            idx = self.fingertip_mapping[finger_name]
                            right_touch_msg.data[14*idx] = tactile_data.timestamp
                            right_touch_msg.data[14*idx+1] = tactile_data.normal_force
                            right_touch_msg.data[14*idx+2] = tactile_data.normal_force_delta
                            right_touch_msg.data[14*idx+3] = tactile_data.tangential_force
                            right_touch_msg.data[14*idx+4] = tactile_data.tangential_force_delta
                            right_touch_msg.data[14*idx+5] = tactile_data.direction
                            right_touch_msg.data[14*idx+6] = tactile_data.proximity
                            right_touch_msg.data[14*idx+7] = tactile_data.temperature
                            right_touch_msg.data[14*idx+8] = tactile_data.encoder1
                            right_touch_msg.data[14*idx+9] = tactile_data.encoder2
                            right_touch_msg.data[14*idx+10] = tactile_data.motor1_error
                            right_touch_msg.data[14*idx+11] = tactile_data.motor2_error
                            right_touch_msg.data[14*idx+12] = tactile_data.impedance1
                            right_touch_msg.data[14*idx+13] = tactile_data.impedance2

                    # Publish tactile data
                    self.right_touch_sensor_pub.publish(right_touch_msg)


        except Exception as e:
            self.logger.error(f"Error processing feedback: {str(e)}")

    def reset_callback(self, request, response):
        """Reset all hands with bend-straighten sequence"""
        try:
            success = True

            # First bend all joints to 30 degrees
            for hand in self.hands.values():
                hand.move_joints(
                    th_dip=30,
                    th_mcp=30,
                    th_rot=30,
                    ff_spr=30,
                    ff_dip=30,
                    ff_mcp=30,
                    mf_dip=30,
                    mf_mcp=30,
                    rf_dip=30,
                    rf_mcp=30,
                    lf_dip=30,
                    lf_mcp=30,
                )
                hand.clear_errors()

            # Wait a moment
            time.sleep(0.5)

            # Then straighten to 0 degrees
            for _ in range(2):
                for hand in self.hands.values():

                    hand.reset_joints()
                    time.sleep(0.005)
                    hand.clear_errors()
                    time.sleep(0.005)


            # Clear command history
            for hand in self.hands:
                self.last_commands[hand] = {}

            # Set response
            response.success = success
            response.message = (
                "Hand reset sequence completed successfully"
                if success
                else "Hand reset sequence failed"
            )
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error in reset sequence: {str(e)}"
            return response

    def on_shutdown(self):
        """Clean up on node shutdown"""
        for hand in self.hands.values():
            hand.close()


def main():
    config_path = os.path.join(os.path.dirname(__file__), "../../config", "config.yaml")

    # Load configuration
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return

    node = DexHandNode(config=config["DexHand"]["ROS_Node"])
    try:
        node.spin()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()