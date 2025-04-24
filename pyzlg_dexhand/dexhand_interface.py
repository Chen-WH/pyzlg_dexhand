from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Union, Tuple
import numpy as np
import os
import yaml
import logging
from pathlib import Path
from typing import List, Dict
import time
from .zcan_wrapper import ZCANWrapper
from . import dexhand_protocol as protocol
from .dexhand_protocol import BoardID
from .dexhand_protocol.commands import (
    ControlMode,
    MotorCommand,
    ClearErrorCommand,
    FeedbackConfigCommand,
    BroadcastCommand,
    encode_broadcast_command,
    FeedbackMode,
    encode_command,
    GlobalCommand,
    GlobalFunctionCode,
)
from .dexhand_protocol.messages import (
    BoardFeedback,
    ErrorInfo,
    MessageType,
    ProcessedMessage,
    FlashStorageTable,
    LogLevel,
    process_message,
)


logger = logging.getLogger(__name__)


@dataclass
class HandConfig:
    """Configuration for hand hardware"""

    channel: int  # CAN channel number
    hall_scale: List[float]  # Scale coefficients for hall position modes


@dataclass
class BoardState:
    """Feedback and Error collected for a single board."""

    feedback_timestamp: float  # Timestamp of last feedback
    status_timestamp: float  # Timestamp of last status (normal / error) update
    is_normal: bool  # True if board is in normal state
    feedback: Optional[BoardFeedback] = None  # Last feedback received
    error_info: Optional[ErrorInfo] = None  # Error information if board is in error state



@dataclass
class JointFeedback:
    """Feedback for a specific joint command"""

    timestamp: float  # When feedback was received
    angle: float  # Joint angle in degrees
    encoder_position: Optional[int] = None  # Encoder position in raw units
    current: Optional[int] = None  # Current in mA
    velocity: Optional[int] = None  # Speed in rpm
    error_code: Optional[int] = None  # Motor error code
    impedance: Optional[float] = None  # Motor impedance value


@dataclass
class StampedTactileFeedback:
    """Timestamped tactile feedback for a fingertip"""

    timestamp: float  # When feedback was received
    normal_force: float  # Normal force in N
    normal_force_delta: int  # Change in normal force (raw units)
    tangential_force: float  # Tangential force in N
    tangential_force_delta: int  # Change in tangential force (raw units)
    direction: int  # Force direction (0-359 degrees, fingertip is 0)
    proximity: int  # Proximity value (raw units)
    temperature: int  # Temperature in Celsius


@dataclass
class HandFeedback:
    """Feedback data for whole hand"""

    query_timestamp: float  # When feedback was requested
    joints: Dict[str, JointFeedback]  # Feedback per joint
    tactile: Dict[str, StampedTactileFeedback]  # Tactile data per fingertip


class DexHandBase:
    """Base class for dexterous hand control"""

    NUM_MOTORS = 12  # Total motors in hand
    NUM_BOARDS = 6  # Number of control boards
    MIN_FIRMWARE_VERSION = 25418  # Minimum recommended firmware version

    joint_names = [
        "th_dip",
        "th_mcp",  # Board 0: Thumb
        "th_rot",
        "ff_spr",  # Board 1: Thumb rotation & spread
        "ff_dip",
        "ff_mcp",  # Board 2: First finger
        "mf_dip",
        "mf_mcp",  # Board 3: Middle finger
        "rf_dip",
        "rf_mcp",  # Board 4: Ring finger
        "lf_dip",
        "lf_mcp",  # Board 5: Little finger
    ]
    finger_map = {
        0: "th",
        2: "ff",
        3: "mf",
        4: "rf",
        5: "lf",
    }  # Map from board index to finger name

    def __init__(
            self,
            config: dict, 
            base_id: int, 
            zcan: Optional[ZCANWrapper] = None, 
            log_level: Optional[LogLevel] = LogLevel.INFO):
        """Initialize dexterous hand interface

        Args:
            config: Path to hand's YAML config file
            base_id: Base board ID (0x01 for left, 0x07 for right)
            zcan: Optional existing ZCANWrapper instance to share between hands
        """
        self.config = HandConfig(
            channel=config["channel"], hall_scale=config["hall_scale"]
        )
        self.base_id = base_id
        self.zcan = zcan if zcan else ZCANWrapper()
        self._owns_zcan = zcan is None
        self.log_level = log_level

        # Hall position scaling factors
        self._init_hall_scaling()

        # Maintain state for each board
        self.board_states: Dict[int, BoardState] = {
            i: BoardState(
                feedback_timestamp=0,
                feedback=None,
                status_timestamp=0,
                is_normal=True,
                error_info=None,
            )
            for i in range(self.NUM_BOARDS)
        }

    def _init_hall_scaling(self):
        """Initialize scaling factors for hall position modes"""
        # Conversion factor for hall position modes (from protocol spec):
        # - 6 counts per revolution
        # - 25:1 gear ratio
        # - 16-bit resolution
        # - 360 degrees per revolution
        factor = 6 * 25 * 2**4 / 360.0  # Converts degrees to hardware units
        self._hall_scale = np.array(self.config.hall_scale) * factor

    def init(self, device_index: int = 0) -> bool:
        """Initialize CAN communication

        Args:
            device_index: Device index for ZCAN device

        Returns:
            bool: True if initialization successful
        """
        if self._owns_zcan:
            if not self.zcan.open(device_index=device_index):
                logger.error("Failed to open CAN device")
                return False

        # Configure channel
        if not self.zcan.configure_channel(self.config.channel):
            logger.error(f"Failed to configure channel {self.config.channel}")
            return False

        return True
        
    def read_flash_memory(self, board_idx: int, address: int, timeout: float = 0.1) -> Optional[bytes]:
        """Read raw data from flash memory
        
        Args:
            board_idx: Board index (0-5)
            address: Memory address to read from
            timeout: Timeout in seconds
            
        Returns:
            Raw bytes read from memory, or None if read failed
        """
        if not 0 <= board_idx < self.NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
            
        if not 0 <= address <= 0xFF:
            raise ValueError(f"Invalid memory address: {address}")
            
        # Create read command 
        board_id = self.base_id + board_idx
        can_id = board_id
        data = bytes([MessageType.COMMAND_READ, address])  # Command read, address
        
        # Send command and wait for response
        response_id = MessageType.CONFIG_RESPONSE + board_id  # Response ID is base + config response base
        self.zcan.send_fd_message(self.config.channel, can_id, data)
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            messages = self.zcan.receive_fd_messages(self.config.channel)
            for msg_id, msg_data, _ in messages:
                if msg_id != response_id:
                    continue
                    
                # Check if this is a valid response for our address
                if len(msg_data) >= 4 and msg_data[0] == MessageType.COMMAND_READ and msg_data[1] == address:
                    # Return the raw data (first two bytes are command and address)
                    return msg_data[2:]
                    
            time.sleep(0.001)
            
        logger.error(f"Failed to read from board {board_idx} address 0x{address:02x}")
        return None
        
    def write_flash_memory(self, board_idx: int, address: int, value: bytes, timeout: float = 0.1) -> bool:
        """Write data to flash memory
        
        Args:
            board_idx: Board index (0-5)
            address: Memory address to write to
            value: Data to write (up to 6 bytes)
            timeout: Timeout in seconds
            
        Returns:
            True if write succeeded, False otherwise
        """
        if not 0 <= board_idx < self.NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
            
        if not 0 <= address <= 0xFF:
            raise ValueError(f"Invalid memory address: {address}")
            
        if len(value) > 6:
            raise ValueError(f"Data too large: maximum 6 bytes, got {len(value)}")
            
        # Create write command
        board_id = self.base_id + board_idx
        can_id = board_id
        
        # Command structure: [write command, address, data...]
        data = bytearray([MessageType.COMMAND_WRITE, address]) + value
        # Pad to 8 bytes total for CAN message
        data.extend([0] * (8 - len(data)))
        
        # Send command and wait for response
        response_id = MessageType.CONFIG_RESPONSE + board_id
        self.zcan.send_fd_message(self.config.channel, can_id, bytes(data))
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            messages = self.zcan.receive_fd_messages(self.config.channel)
            for msg_id, msg_data, _ in messages:
                if msg_id != response_id:
                    continue
                    
                # Check if this is a valid response
                if len(msg_data) >= 5 and msg_data[0] == MessageType.COMMAND_WRITE and msg_data[1] == address:
                    # Check for success indicator at byte 4
                    success = msg_data[4] == 0x01
                    if not success:
                        logger.error(f"Write to board {board_idx} address 0x{address:02x} failed")
                    return success
                    
            time.sleep(0.001)
            
        logger.error(f"No response to write to board {board_idx} address 0x{address:02x}")
        return False
        
    def get_board_firmware_version(self, board_idx: int = 0) -> Optional[int]:
        """Get the firmware version from a specific board
        
        Reads the firmware version value from memory address 0x02 of the specified board.
        
        Args:
            board_idx: Board index (0-5)
            
        Returns:
            Firmware version number, or None if read failed
        """
        data = self.read_flash_memory(board_idx, FlashStorageTable.MEMORY_ADDRESS_FIRMWARE_VERSION)
        if not data or len(data) < 2:
            return None
            
        # Extract firmware version (little-endian)
        return int.from_bytes(data[:2], 'little')
        
    def save_to_flash(self, board_idx: int) -> bool:
        """Save current configuration to flash memory
        
        This command persists any configuration changes to flash memory so they
        will survive power cycles. Without this, configuration changes are temporary.
        
        Args:
            board_idx: Board index (0-5)
            
        Returns:
            True if save succeeded, False otherwise
        """
        # The save-to-flash command is a write to address 0x04 with no data
        return self.write_flash_memory(board_idx, FlashStorageTable.MEMORY_ADDRESS_SAVE_TO_FLASH, b'')
        
    def get_firmware_versions(self) -> Dict[str, Optional[int]]:
        """Get firmware versions from all boards
        
        Returns a dictionary mapping joint names to their firmware versions.
        Boards controlling multiple joints (like the thumb board) will have the same
        firmware version reported for all joints controlled by that board.
        
        Returns:
            Dictionary mapping joint names to firmware versions, or None if read failed
        """
        # Map from board index to joint names
        board_to_joints = {
            0: ["th_dip", "th_mcp"],           # Thumb board
            1: ["th_rot", "ff_spr"],           # Thumb rotation & spread board
            2: ["ff_dip", "ff_mcp"],           # First finger board  
            3: ["mf_dip", "mf_mcp"],           # Middle finger board
            4: ["rf_dip", "rf_mcp"],           # Ring finger board
            5: ["lf_dip", "lf_mcp"],           # Little finger board
        }
        
        versions = {}
        
        # For each board, fetch its firmware version and assign to all joints on that board
        for board_idx in range(self.NUM_BOARDS):
            version = self.get_board_firmware_version(board_idx)
            for joint_name in board_to_joints.get(board_idx, []):
                versions[joint_name] = version
                
        return versions

    def _get_command_id(self, msg_type: MessageType, board_idx: int) -> int:
        """Get command CAN ID for a board index"""
        if not 0 <= board_idx < self.NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
        return msg_type + self.base_id + board_idx

    def set_feedback_mode(
        self, mode: FeedbackMode, period_ms: int, enable: bool
    ) -> bool:
        """Configure feedback mode for all boards

        Args:
            mode: Feedback mode
            period_ms: Period in milliseconds (if periodic)
            enable: Enable flag

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = FeedbackConfigCommand(mode=mode, period_ms=period_ms, enable=enable)

        try:
            msg_type, data = encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode feedback config command: {e}")
            return False

        # Send command to all boards
        for board_idx in range(self.NUM_BOARDS):
            command_id = self._get_command_id(msg_type, board_idx)
            if not self.zcan.send_fd_message(self.config.channel, command_id, data):
                logger.error(
                    f"Failed to send feedback config command to board {board_idx}"
                )
                return False

        return True

    def set_safe_temperature(
            self, 
            safe_temperature: int = None,
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the security temperature to prevent overheating (default value is 55℃)

        Args:
            safe_temperature (Uint8): Safety temperature value, ranging from 0 to 255

        Returns:
            bool: True if command sent successfully
        """
        try:
            if safe_temperature is None or not (0 <= safe_temperature <= 255):
                logger.error(f"Invalid safe temperature: {safe_temperature}")
                return False
            
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}:
                logger.error(f"Invalid log level: {log_level}")
                return False
            
        except ValueError as e:
            logger.error(f"Invalid safe temperature: {e}")
            return
        
        # Construct write command
        data = safe_temperature.to_bytes(1, byteorder='little')
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_SAFE_TEMPERATURE]) + data
        
        # Send command
        success = self._send_command(command)

        if log_level is not None:
            if log_level <= LogLevel.DEBUG:
                logger.debug("Command sent successfully for set safe temperature: {safe_temperature}")
        elif self.log_level <= LogLevel.DEBUG:
            logger.debug("Command sent successfully for set safe temperature: {safe_temperature}")
        return success

    # this usage has been deprecated
    def current_motor_control_torque(
            self, 
            motor_type: str, 
            current: int,
            log_level: Optional[LogLevel] = None) -> bool:
        """
        ***this usage has been deprecated***
        Set the motor torque.

        Args:
            motor_type (str): Motor type, can only be "motor1", "motor2", or "motor"
            current (int): Torque value, ranging from 0 to 599

        Returns:
            bool: Returns True if set successfully, otherwise returns False.
        """
        try:
            if motor_type not in ["motor1", "motor2", "motor"]:
                logger.error(f"Invalid motor type: {motor_type}")
                return False

            if not (0 <= current <= 599):
                logger.error(f"Invalid current value: {current}")
                return False
            
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}:
                logger.error(f"Invalid log level: {log_level}")
                return False

            # Select memory address according to motor type
            if motor_type == "motor1":
                address = FlashStorageTable.MEMORY_ADDRESS_MOTOR1_TORQUE
                data = current.to_bytes(2, byteorder='little')
            elif motor_type == "motor2":
                address = FlashStorageTable.MEMORY_ADDRESS_MOTOR2_TORQUE
                data = current.to_bytes(2, byteorder='little')
            elif motor_type == "motor":
                address = FlashStorageTable.MEMORY_ADDRESS_BOTH_MOTORS_TORQUE
                data = current.to_bytes(4, byteorder='little') 

        except ValueError as e:
            logger.error(f"Invalid motor type or current value: {e}")
            return
        # Construct write command
        command = bytes([MessageType.COMMAND_WRITE, address]) + data

        # Send command
        success = self._send_command(command)
        if log_level is not None:
            if log_level <= LogLevel.DEBUG:
                logger.debug("Command sent successfully for {motor_type} control torque: {current}")
        elif self.log_level <= LogLevel.DEBUG:
            logger.debug("Command sent successfully for {motor_type} control torque: {current}")
        return success

    def set_stall_time(
            self, 
            motor_type: str,
            stall_time: int,
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the stall time (optional).

        Args:
            stall_time (int): The stall time in milliseconds

        Returns:
            bool: Returns True if set successfully, False otherwise
        """

        try:
            if motor_type not in ["motor1", "motor2", "motor"]:
                logger.error(f"Invalid motor type: {motor_type}")
                return False

            if not (0 <= stall_time <= 65535):
                logger.error(f'stall time out of range')
                return False
            
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}:
                logger.error(f"Invalid log level: {log_level}")
                return False
        except ValueError as e:
            logger.error(f"Invalid stall time: {e}")
            return

        # Construct write command
        data = stall_time.to_bytes(2, byteorder='little') 
        if motor_type == "motor1":
            command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR1]) + data
            success = self._send_command(command)
        elif motor_type == "motor2":
            command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR2]) + data
            success = self._send_command(command)
        else:
            command1 = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR1]) + data
            command2 = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR2]) + data
            # Send command
            success = self._send_command(command1) and self._send_command(command2)
        if log_level is not None:
            if log_level <= LogLevel.DEBUG:
                logger.debug("Command sent successfully for {motor_type} stall time: {stall_time}")
        elif self.log_level <= LogLevel.DEBUG:
            logger.debug("Command sent successfully for set {motor_type} stall time: {stall_time}")
        return success
    
    def set_pressure_limit_value(
            self, 
            value: int , 
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the pressure limit value.

        Args:
            value (int): Pressure limit value, ranging from 0 to 20 N
            log_level (LogLevel): Logging level

        Returns:
            bool: Returns True if set successfully, False otherwise
        """
        try:
            if not (0 <= value <= 20):
                logger.error(f"Invalid pressure limit value: {value}")
                return False
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR,None}:
                logger.error(f"Invalid log level: {log_level}")
                return False
        except ValueError as e:
            logger.error(f"Invalid pressure limit value: {e}")
            return

        # Construct write command
        value = value * 100
        data = value.to_bytes(2, byteorder='little')
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_VALUE]) + data

        # Send command
        success = self._send_command(command)
        if log_level is not None:
            if log_level <= LogLevel.DEBUG:
                logger.debug("Command sent successfully for set pressure limit value: {value}")
        elif self.log_level <= LogLevel.DEBUG:
            logger.debug("Command sent successfully for set pressure limit value: {value}")
        return success
    
    def set_pressure_limit_enable(
        self, 
        enable: bool, 
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """
        Enable/disable the pressure limit function.

        Args:
            enable (bool): True to enable pressure limit, False to disable
            log_level (LogLevel): Logging level for operation feedback

        Returns:
            bool: True if command executed successfully, False otherwise
        """
        try:
            if not isinstance(enable, bool):
                logger.error(f"Invalid pressure limit enable: {enable}")
                return False
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR , None}:
                logger.error(f"Invalid log level: {log_level}")
                return False
        except ValueError as e:
            logger.error(f"Invalid pressure limit enable: {e}")
            return
        
        data = 1 if enable else 0
        data = data .to_bytes(1, byteorder='little')
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_ENABLE]) + data
        # Send command
        success = self._send_command(command)
        if log_level is not None:
            if log_level <= LogLevel.DEBUG:
                logger.debug("Command sent successfully for set pressure limit enable: {enable}")
        elif self.log_level <= LogLevel.DEBUG:
            logger.debug("Command sent successfully for set pressure limit enable: {enable}")
        return success

    def _send_command(self, command: bytes,log_level: Optional[LogLevel] = None) -> bool:
        """
        Send a command to the control board.

        Args:
            command (bytes): The command data to be sent.

        Returns:
            bool: Returns True if set successfully, False otherwise
        """
        try:
            if log_level is not None:
                if log_level <= LogLevel.DEBUG:
                    # Record the original instruction data sent
                    logger.debug(f"Sending command: {command.hex()}")
            elif self.log_level <= LogLevel.DEBUG:
                logger.debug(f"Sending command: {command.hex()}")
            # Send commands to all boards
            for board_idx in range(self.NUM_BOARDS):
                command_id = self._get_command_id(MessageType.CONFIG_COMMAND, board_idx)
                if not self.zcan.send_fd_message(self.config.channel, command_id, command):
                    logger.error(f"Failed to send command to board {board_idx}")
                    return False
            return True
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False

    def _send_motion_command(
        self,
        board_idx: int,
        motor1_pos: int,
        motor2_pos: int,
        motor_enable: int = 0x03,
        control_mode: ControlMode = ControlMode.IMPEDANCE_GRASP,
        motor1_speed: Optional[int] = None,
        motor2_speed: Optional[int] = None,
        motor1_current: Optional[int] = None,
        motor2_current: Optional[int] = None,
    ) -> bool:
        """Send a motion command to a specific board

        Args:
            board_idx: Board index to command
            motor1_pos: Position command for motor 1, in hardware units
            motor2_pos: Position command for motor 2, in hardware units
            motor_enable: Motor enable flags, 0x01 for motor 1, 0x02 for motor 2, 0x03 for both
            control_mode: Control mode

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = MotorCommand(
            control_mode=control_mode,
            motor_enable=motor_enable,
            motor1_pos=motor1_pos,
            motor2_pos=motor2_pos,
            motor1_speed=motor1_speed,
            motor2_speed=motor2_speed,
            motor1_current=motor1_current,
            motor2_current=motor2_current,
        )

        try:
            msg_type, data = encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode command: {e}")
            return False

        # Send command
        command_id = self._get_command_id(MessageType.MOTION_COMMAND, board_idx)
        if not self.zcan.send_fd_message(self.config.channel, command_id, data):
            logger.error("Failed to send command to ID {command_id}")
            return False

        return True
    
    def send_broadcast_control_frame(
        self,
        control_mode: ControlMode,
        enable_motors: List[bool] = None,  # 12 booleans for each motor
        clear_error: bool = False,
        request_feedback: bool = True,
        is_right_hand: bool = False,
        positions: List[int] = None,  # 12 positions corresponding to each motor
        speeds: List[int] = None,     # 12 speeds corresponding to each motor
        currents: List[int] = None,   # 12 currents corresponding to each motor
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send a broadcast control frame to control all motors at once with CAN ID 0x100.
        
        Args:
            control_mode: Control mode enum
            enable_motors: List of 12 booleans indicating which motors to enable, in order:
                [th_dip, th_mcp, th_rot, ff_spr, ff_dip, ff_mcp, mf_dip, mf_mcp, rf_dip, rf_mcp, lf_dip, lf_mcp]
            clear_error: Whether to clear errors
            request_feedback: Whether to request feedback
            is_right_hand: True for right hand, False for left hand
            positions: List of 12 position values corresponding to each motor (-32768 to 32767)
            speeds: List of 12 speed values corresponding to each motor (0 to 32767)
            currents: List of 12 current values corresponding to each motor (10 to 599 mA)
            log_level: Logging level for this operation
                    
        Returns:
            bool: True if command sent successfully
        """
        # Default to all motors enabled if not specified
        if enable_motors is None:
            enable_motors = [True] * 12
        
        # Default values for positions, speeds, currents
        if positions is None:
            positions = [0] * 12
        if speeds is None:
            speeds = [15000] * 12  # Default speed: 15000
        if currents is None:
            currents = [20] * 12   # Default current: 20mA
        
        # Create broadcast command
        command = BroadcastCommand(
            control_mode=control_mode,
            enable_motors=enable_motors,
            clear_error=clear_error,
            request_feedback=request_feedback,
            is_right_hand=is_right_hand,
            positions=positions,
            speeds=speeds,
            currents=currents
        )
        
        try:
            # Encode command using the function from commands.py
            frame_data = encode_broadcast_command(command)
            
            # Log debug information if requested
            if (log_level is not None and log_level <= LogLevel.DEBUG) or self.log_level <= LogLevel.DEBUG:
                logger.debug(f"Sending broadcast control frame: {frame_data.hex()}")
            
            # Send the frame with ID 0x100
            if not self.zcan.send_fd_message(self.config.channel, 0x100, frame_data):
                logger.error("Failed to send broadcast control frame")
                return False
            
            # Log success information if requested
            if (log_level is not None and log_level <= LogLevel.INFO) or self.log_level <= LogLevel.INFO:
                logger.info("Successfully sent broadcast control frame")
                
            return True
            
        except ValueError as e:
            logger.error(f"Failed to encode broadcast command: {e}")
            return False
        
    def send_global_command(
        self,
        function_code: GlobalFunctionCode, 
        data: bytes = b'', 
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send a global broadcast command to all boards.
        
        Args:
            function_code: The global function code
            data: Optional data for the command (max 6 bytes)
            log_level: Logging level for this operation
                    
        Returns:
            bool: True if command sent successfully
        """
        # 
        command = GlobalCommand(function_code=function_code, data=data)
        
        try:
            # Encode command using the function from commands.py
            msg_type, cmd_data = encode_command(command)
            
            # Log debug information if requested
            if (log_level is not None and log_level <= LogLevel.DEBUG) or self.log_level <= LogLevel.DEBUG:
                logger.debug(f"Sending global command: function_code={function_code}, data={data.hex() if data else 'None'}")
            
            # Send the command
            if not self.zcan.send_fd_message(self.config.channel, msg_type, cmd_data):
                logger.error(f"Failed to send global command: {function_code}")
                return False
            
            # Log success information if requested
            if (log_level is not None and log_level <= LogLevel.INFO) or self.log_level <= LogLevel.INFO:
                logger.info(f"Successfully sent global command: {function_code}")
                
            return True
            
        except ValueError as e:
            logger.error(f"Failed to encode global command: {e}")
            return False

    def _refresh_board_states(self,log_level: Optional[LogLevel] = None):
        """Receive CANFD frames to update the states for all boards."""
        # Get all messages
        messages = self.zcan.receive_fd_messages(self.config.channel)

        # Record the received original feedback data
        if log_level is not None and log_level <= LogLevel.DEBUG:
            for msg_id, data, timestamp in messages:
                logger.debug(f"Received original feedback: msg_id={msg_id}, data={data.hex()}, timestamp={timestamp}")

        # Process all received messages
        elif self.log_level <= LogLevel.DEBUG:
            for msg_id, data, timestamp in messages:
                logger.debug(f"Received feedback: msg_id={msg_id}, data={data.hex()}, timestamp={timestamp}")

        # Process all received messages
        for msg_id, data, timestamp in messages:
            try:
                result = process_message(msg_id, data)
                board_idx = msg_id - result.msg_type - self.base_id

                if result.msg_type == MessageType.MOTION_FEEDBACK:
                    self.board_states[board_idx].feedback_timestamp = timestamp
                    self.board_states[board_idx].feedback = result.feedback
                elif result.msg_type == MessageType.ERROR_MESSAGE:
                    self.board_states[board_idx].status_timestamp = timestamp
                    self.board_states[board_idx].is_normal = False
                    self.board_states[board_idx].error_info = result.error_info
                elif result.msg_type == MessageType.CONFIG_RESPONSE:
                    success, command_type = protocol.messages.verify_config_response(
                        msg_id, data
                    )
                    if (
                        success
                        and command_type == protocol.commands.CommandType.CLEAR_ERROR
                    ):
                        self.board_states[board_idx].error_info = None
                        self.board_states[board_idx].is_normal = True
            except ValueError as e:
                logger.error(f"Failed to process message: {e}")

    def _clear_board_error(self, board_idx: int) -> bool:
        """Attempt to clear error state for a board

        Args:
            board_idx: Board index to clear error for

        Returns:
            bool: True if error clearance command sent successfully
        """
        # Create and encode clear error command
        clear_cmd = ClearErrorCommand()
        msg_type, clear_data = encode_command(clear_cmd)
        clear_cmd_id = self._get_command_id(msg_type, board_idx)

        # Send command
        if not self.zcan.send_fd_message(self.config.channel, clear_cmd_id, clear_data):
            logger.error(f"Failed to send error clear command to board {board_idx}")
            return False

        return True
    def move_joints(
        self,
        th_rot: Optional[float] = None,  # thumb rotation
        th_mcp: Optional[float] = None,  # thumb metacarpophalangeal
        th_dip: Optional[float] = None,  # thumb coupled distal joints
        ff_spr: Optional[float] = None,  # four-finger spread
        ff_mcp: Optional[float] = None,  # first finger metacarpophalangeal
        ff_dip: Optional[float] = None,  # first finger coupled distal joints
        mf_mcp: Optional[float] = None,  # middle finger metacarpophalangeal
        mf_dip: Optional[float] = None,  # middle finger coupled distal joints
        rf_mcp: Optional[float] = None,  # ring finger metacarpophalangeal
        rf_dip: Optional[float] = None,  # ring finger coupled distal joints
        lf_mcp: Optional[float] = None,  # little finger metacarpophalangeal
        lf_dip: Optional[float] = None,  # little finger coupled distal joints
        control_mode: ControlMode = ControlMode.IMPEDANCE_GRASP,  # Control mode
        speeds: Union[int, List[int]] = 15000,  # Speed for all motors or list of speeds
        currents: Union[int, List[int]] = 20,  # Current for all motors or list of currents
        use_broadcast: bool = False,  # Whether to use broadcast command (more efficient)
        clear_error: bool = False,  # Whether to clear errors (only for broadcast mode)
        request_feedback: bool = True,  # Whether to request feedback (only for broadcast mode)
        log_level: Optional[LogLevel] = None,  # Log level
    ):
        """Move hand joints to specified angles.

        For each finger, there are two independent DOFs:
        - MCP (metacarpophalangeal) joint: Controls base joint flexion
        - DIP (coupled): Controls coupled motion of PIP and DIP joints

        Additional DOFs:
        - th_rot: Thumb rotation/opposition
        - ff_spr: Four-finger spread (abduction between fingers)

        Args:
            th_rot: Thumb rotation angle in degrees
            th_mcp: Thumb MCP flexion angle
            th_dip: Thumb coupled PIP-DIP flexion
            ff_spr: Four-finger spread angle
            ff_mcp: Index MCP flexion
            ff_dip: Index coupled PIP-DIP flexion
            mf_mcp: Middle MCP flexion
            mf_dip: Middle coupled PIP-DIP flexion
            rf_mcp: Ring MCP flexion
            rf_dip: Ring coupled PIP-DIP flexion
            lf_mcp: Little MCP flexion
            lf_dip: Little coupled PIP-DIP flexion
            control_mode: Motor control mode
            speeds: Either a single speed value for all motors (0-32767) or a list of 12 speed values
            currents: Either a single current value for all motors (10-599mA) or a list of 12 current values
            use_broadcast: If True, send a single broadcast command for all joints (more efficient)
            clear_error: Whether to clear errors (only for broadcast mode)
            request_feedback: Whether to request feedback (only for broadcast mode)
            log_level: Logging level for this operation
        """
        # Record command start time
        command_timestamp = time.time()

        # Map joint angles to motor commands
        motor_angles = [
            th_dip,
            th_mcp,  # Board 0
            th_rot,
            ff_spr,  # Board 1
            ff_dip,
            ff_mcp,  # Board 2
            mf_dip,
            mf_mcp,  # Board 3
            rf_dip,
            rf_mcp,  # Board 4
            lf_dip,
            lf_mcp,  # Board 5
        ]

        # Create enable_motors list based on which angles are provided
        enable_motors = [angle is not None for angle in motor_angles]
        
        # Scale angles for the specified control mode
        scaled_positions = []
        for i, angle in enumerate(motor_angles):
            if angle is not None:
                scaled_positions.append(int(self._scale_angle(i, angle, control_mode)))
            else:
                scaled_positions.append(0)
        
        # Handle speed parameter
        if isinstance(speeds, int):
            motor_speeds = [speeds] * 12
        elif len(speeds) == 12:
            motor_speeds = speeds
        else:
            logger.error(f"If speeds is a list, it must contain exactly 12 values")
            return False
            
        # Handle current parameter
        if isinstance(currents, int):
            motor_currents = [currents] * 12
        elif len(currents) == 12:
            motor_currents = currents
        else:
            logger.error(f"If currents is a list, it must contain exactly 12 values")
            return False
        
        # Use broadcast mode if requested (more efficient)
        if use_broadcast:
            # Use right hand if this is a RightDexHand instance
            is_right_hand = isinstance(self, RightDexHand)
            
            # Send the broadcast command
            return self.send_broadcast_control_frame(
                control_mode=control_mode,
                enable_motors=enable_motors,
                positions=scaled_positions,
                speeds=motor_speeds,
                currents=motor_currents,
                clear_error=clear_error,
                request_feedback=request_feedback,
                is_right_hand=is_right_hand,
                log_level=log_level
            )
        else:
            # Use traditional per-board commands
            success = True
            for board_idx in range(self.NUM_BOARDS):
                base_idx = board_idx * 2
                if any(enable_motors[base_idx:base_idx + 2]):
                    motor_enable = 0x01 if enable_motors[base_idx] else 0
                    motor_enable |= 0x02 if enable_motors[base_idx + 1] else 0
                    board_success = self._send_motion_command(
                        board_idx=board_idx,
                        motor1_pos=scaled_positions[base_idx],
                        motor2_pos=scaled_positions[base_idx + 1],
                        motor_enable=motor_enable,
                        control_mode=control_mode,
                        motor1_speed=motor_speeds[base_idx],
                        motor2_speed=motor_speeds[base_idx + 1],
                        motor1_current=motor_currents[base_idx],
                        motor2_current=motor_currents[base_idx + 1],
                    )
                    if not board_success:
                        logger.error(f"Failed to send command to board {board_idx}")
                        success = False
                        
            # Log success if requested
            if success:
                if (log_level is not None and log_level <= LogLevel.INFO) or self.log_level <= LogLevel.INFO:
                    logger.info(f"Successfully executed move_joints command")
                    
            return success
            
    def get_feedback(self) -> HandFeedback:
        """Get feedback from all joints and tactile sensors

        Returns:
            HandFeedback object.
        """
        # Record query start time
        query_timestamp = time.time_ns() / 1e9

        # Refresh board states to get feedback
        self._refresh_board_states()

        # Process feedback from all boards
        joint_feedback = {}
        tactile_feedback = {}
        for board_idx, state in self.board_states.items():
            base_idx = board_idx * 2
            timestamp_feedback = time.time_ns()

            if state.feedback is None:
                # No feedback available
                for i in range(2):
                    joint_idx = base_idx + i
                    joint_feedback[self.joint_names[joint_idx]] = JointFeedback(
                        timestamp=timestamp_feedback,
                        angle=float("nan"),
                        encoder_position=None,
                        current=None,
                        velocity=None,
                        error_code=None,
                        impedance=None
                    )
                continue

            # Process joint feedback
            motors = [state.feedback.motor1, state.feedback.motor2]
            for i in range(2):
                joint_idx = base_idx + i
                joint_feedback[self.joint_names[joint_idx]] = JointFeedback(
                    timestamp=timestamp_feedback,
                    angle=motors[i].angle,
                    encoder_position=motors[i].position,
                    current=motors[i].current,
                    velocity=motors[i].velocity,
                    error_code=getattr(motors[i], 'error_code', None),
                    impedance=getattr(motors[i], 'impedance', None)
                )

            # Process tactile feedback if available
            if state.feedback.tactile is not None:
                if board_idx in self.finger_map:
                    tactile = state.feedback.tactile
                    tactile_feedback[self.finger_map[board_idx]] = StampedTactileFeedback(
                        timestamp=timestamp_feedback,
                        normal_force=tactile.normal_force,
                        normal_force_delta=tactile.normal_force_delta,
                        tangential_force=tactile.tangential_force,
                        tangential_force_delta=tactile.tangential_force_delta,
                        direction=tactile.direction,
                        proximity=tactile.proximity,
                        temperature=tactile.temperature
                    )

        return HandFeedback(
            query_timestamp=query_timestamp,
            joints=joint_feedback,
            tactile=tactile_feedback,
        )

    def get_errors(self) -> Dict[int, Optional[ErrorInfo]]:
        """Get error information for whole hand

        Returns:
            Dict mapping board index to ErrorInfo if an error is present
        """
        return {i: state.error_info for i, state in self.board_states.items()}

    def clear_errors(self, clear_all=True, use_global=True, log_level: Optional[LogLevel] = None) -> bool:
        """Clear errors for the hand
        
        Args:
            clear_all: If True, attempt to clear errors for all boards even if not in error state
            use_global: If True, use more efficient global command to clear all errors
            log_level: Optional logging level for the operation
            
        Returns:
            bool: True if commands sent successfully
        """
        # Use the more efficient global command when clearing all errors
        if clear_all and use_global:
            return self.send_global_command(GlobalFunctionCode.CLEAR_ERROR, log_level=log_level)
        
        # Fall back to individual commands for selective clearing
        success = True
        for board_idx in range(self.NUM_BOARDS):
            if clear_all or not self.board_states[board_idx].is_normal:
                if not self._clear_board_error(board_idx):
                    success = False
        
        return success
    
    def clear_all_errors(self, log_level: Optional[LogLevel] = None) -> bool:
        """Clear all errors for the hand using global command (faster)
        
        DEPRECATED: Use clear_errors(use_global=True) instead.
        
        Args:
            log_level: Optional logging level for the operation
            
        Returns:
            bool: True if command sent successfully
        """
        import warnings
        warnings.warn(
            "clear_all_errors is deprecated, use clear_errors(use_global=True) instead", 
            DeprecationWarning, 
            stacklevel=2
        )
        return self.clear_errors(clear_all=True, use_global=True, log_level=log_level)

    def _scale_angle(
        self, motor_idx: int, angle: float, control_mode: ControlMode
    ) -> int:
        """Scale angle based on control mode"""
        if control_mode in (
            ControlMode.HALL_POSITION,
            ControlMode.PROTECT_HALL_POSITION,
        ):
            return int(angle * self._hall_scale[motor_idx])
        else:
            # For cascaded PID mode, scale to 100x for hardware units
            return int(angle * 100)

    def reset_joints(self, use_broadcast: bool = False, log_level: Optional[LogLevel] = None):
        """Reset all joints to their zero positions.

        This is equivalent to setting all joint angles to 0 degrees.
        Uses CASCADED_PID control mode.

        Args:
            use_broadcast: If True, use more efficient broadcast mode
            log_level: Logging level for the operation
        """
        return self.move_joints(
            th_rot=0,
            th_mcp=0,
            th_dip=0,
            ff_spr=0,
            ff_mcp=0,
            ff_dip=0,
            mf_mcp=0,
            mf_dip=0,
            rf_mcp=0,
            rf_dip=0,
            lf_mcp=0,
            lf_dip=0,
            control_mode=ControlMode.CASCADED_PID,
            use_broadcast=use_broadcast,
            log_level=log_level
        )

    def close(self):
        """Close CAN communication"""
        if self._owns_zcan:
            self.zcan.close()


class LeftDexHand(DexHandBase):
    """Control interface for left dexterous hand"""

    def __init__(self, zcan: Optional[ZCANWrapper] = None, log_level: Optional[LogLevel] = LogLevel.INFO):
        config_path = os.path.join(
            os.path.dirname(__file__), "../config", "config.yaml"
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        super().__init__(
            config["DexHand"]["left_hand"],
            BoardID.LEFT_HAND_BASE,
            zcan,
            log_level=log_level
        )


class RightDexHand(DexHandBase):
    """Control interface for right dexterous hand"""

    def __init__(self, zcan: Optional[ZCANWrapper] = None, log_level: Optional[LogLevel] = LogLevel.INFO):
        config_path = os.path.join(
            os.path.dirname(__file__), "../config", "config.yaml"
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        super().__init__(
            config["DexHand"]["right_hand"],
            BoardID.RIGHT_HAND_BASE,
            zcan,
            log_level=log_level
        )
