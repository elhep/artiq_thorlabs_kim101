"""
Thorlabs KIM101 K-Cube Inertial Piezo Motor Controller Driver

This driver provides a Python interface for controlling the Thorlabs KIM101
using the serial communication protocol.

Author: Python Driver Implementation
Date: 2026-01-31
Protocol Version: Issue 44.1
"""

import serial
import struct
import time
from enum import IntEnum
from typing import Optional, Tuple, List, Any
from dataclasses import dataclass
import abc
import logging
from kim101_driver_I import *


class KIM101Error(Exception):
    """Base exception for KIM101 driver errors"""
    pass


class Channel(IntEnum):
    """Channel identifiers for KIM101"""
    CHANNEL_1 = 0x01
    CHANNEL_2 = 0x02
    CHANNEL_3 = 0x04
    CHANNEL_4 = 0x08


class JogMode(IntEnum):
    """Jog mode options"""
    CONTINUOUS = 0x01
    SINGLE_STEP = 0x02


class TriggerMode(IntEnum):
    """Trigger I/O modes"""
    DISABLED = 0x00
    GPI = 0x01
    RELMOVE = 0x02
    ABSMOVE = 0x03
    RESETCOUNT = 0x04
    GPO = 0x0A
    INMOTION = 0x0B
    MAXVELOCITY = 0x0C
    POSSTEPS_FWD = 0x0D
    POSSTEPS_REV = 0x0E
    POSSTEPS_BOTH = 0x0F
    FWDLIMIT = 0x10
    REVLIMIT = 0x11
    EITHERLIMIT = 0x12


class TriggerPolarity(IntEnum):
    """Trigger polarity"""
    HIGH = 0x01
    LOW = 0x02


class LimitSwitchMode(IntEnum):
    """Limit switch modes"""
    IGNORE = 0x01
    MAKES_ON_CONTACT = 0x02
    BREAKS_ON_CONTACT = 0x03
    MAKES_HOME = 0x04
    BREAKS_HOME = 0x05


class FeedbackSignalMode(IntEnum):
    """Feedback signal modes"""
    DISABLED = 0x00
    LIMSWITCH = 0x01
    ENCODER = 0x02


class JoystickMode(IntEnum):
    """Joystick operating modes"""
    VELOCITY_CONTROL = 0x01
    JOG = 0x02
    GOTO_POSITION = 0x03


class JoystickDirection(IntEnum):
    """Joystick direction sense"""
    DISABLED = 0x00
    NORMAL = 0x01
    REVERSED = 0x02


class ChannelEnableMode(IntEnum):
    """Channel enable modes"""
    NONE = 0x00
    CHANNEL_1 = 0x01
    CHANNEL_2 = 0x02
    CHANNEL_3 = 0x03
    CHANNEL_4 = 0x04
    CHANNELS_1_2 = 0x05
    CHANNELS_3_4 = 0x06


class KIM101(KIM101Interface):
    """
    Thorlabs KIM101 K-Cube Inertial Piezo Motor Controller Driver
    
    This class provides a comprehensive interface for controlling the KIM101
    controller via serial communication.
    """
    
    # Message IDs
    MSG_HW_REQ_INFO = 0x0005
    MSG_HW_GET_INFO = 0x0006
    MSG_HW_START_UPDATEMSGS = 0x0011
    MSG_HW_STOP_UPDATEMSGS = 0x0012
    
    MSG_PZMOT_SET_PARAMS = 0x08C0
    MSG_PZMOT_REQ_PARAMS = 0x08C1
    MSG_PZMOT_GET_PARAMS = 0x08C2
    MSG_PZMOT_MOVE_ABSOLUTE = 0x08D4
    MSG_PZMOT_MOVE_COMPLETED = 0x08D6
    MSG_PZMOT_MOVE_JOG = 0x08D9
    MSG_PZMOT_REQ_STATUSUPDATE = 0x08E0
    MSG_PZMOT_GET_STATUSUPDATE = 0x08E1
    
    # Sub-message IDs
    SUBMSG_PZMOT_POSCOUNTS = 0x0005
    SUBMSG_PZMOT_DRIVEOP_PARAMS = 0x0007
    SUBMSG_PZMOT_LIMSWITCH_PARAMS = 0x000B
    SUBMSG_PZMOT_HOME_PARAMS = 0x000F
    SUBMSG_PZMOT_KCUBEMMI_PARAMS = 0x0015
    SUBMSG_PZMOT_TRIGIO_CONFIG = 0x0017
    SUBMSG_PZMOT_TRIG_PARAMS = 0x0019
    SUBMSG_PZMOT_CHANENABLE_MODE = 0x002B
    SUBMSG_PZMOT_KCUBEJOG_PARAMS = 0x002D
    
    SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS = 0x0030
    SUBMSG_PZMOT_MOVERELATIVE_PARAMS = 0x0032
    SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS = 0x0034
    
    # Device identifiers
    DEST_GENERIC_USB = 0x50
    SOURCE_GENERIC_USB = 0x01

    # TODO rework all I/O values to simple types, which can be parsed with PYON
    
    def __init__(self, device: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize KIM101 controller connection.
        
        Args:
            device: Serial device name (e.g., 'COM3' or '/dev/ttyUSB0')
            baudrate: Communication baud rate (default: 115200)
            timeout: Serial timeout in seconds (default: 1.0)
        """
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._status_update_active = False
        self.connect()
        
    def connect(self) -> None:
        """Open serial connection to the controller."""
        try:
            self.serial = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
        except serial.SerialException as e:
            raise KIM101Error(f"Failed to connect to {self.device}: {e}")
    
    def close(self) -> None:
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.stop_status_updates()
            self.serial.close()
    
    def _send_message(self, msg_id: int, data: bytes = b'',
                     dest: int = DEST_GENERIC_USB, 
                     source: int = SOURCE_GENERIC_USB) -> None:
        """
        Send a message to the controller.
        
        Args:
            msg_id: Message ID (2 bytes)
            data: Data payload
            dest: Destination byte
            source: Source byte
        """
        if not self.serial or not self.serial.is_open:
            raise KIM101Error("Serial connection not open")
        
        # Build header
        msg_id_bytes = struct.pack('<H', msg_id)
        
        if len(data) == 0:
            # Short message format (6 bytes)
            header = msg_id_bytes + struct.pack('BBBx', 0, 0, dest) + struct.pack('B', source)
            message = header
        else:
            # Long message format
            data_len = len(data)
            header = msg_id_bytes + struct.pack('<HBB', data_len, dest | 0x80, source)
            message = header + data
        
        self.serial.write(message)
    
    def _send_short_message(self, msg_id: int, param1: int, param2: int,
                            dest: int = DEST_GENERIC_USB,
                            source: int = SOURCE_GENERIC_USB) -> None:
        """
        Send a short message to the controller.
        
        Args:
            msg_id: Message ID (2 bytes)
            param1: 1st parameter byte
            param2: 2nd parameter byte
            dest: Destination byte
            source: Source byte
        """
        if not self.serial or not self.serial.is_open:
            raise KIM101Error("Serial connection not open")
        msg = struct.pack('<HBBBB', msg_id, param1, param2, dest, source)
        self.serial.write(msg)
    
    def _receive_message(self, expected_msg_id: Optional[int] = None, 
                        timeout: Optional[float] = None) -> Tuple[int, bytes]:
        """
        Receive a message from the controller.
        
        Args:
            expected_msg_id: Expected message ID (or None to accept any)
            timeout: Override default timeout
            
        Returns:
            Tuple of (message_id, data)
        """
        if not self.serial or not self.serial.is_open:
            raise KIM101Error("Serial connection not open")
        
        old_timeout = self.serial.timeout
        if timeout is not None:
            self.serial.timeout = timeout
        
        try:
            # Read header (6 bytes minimum)
            header = self.serial.read(6)
            if len(header) < 6:
                raise KIM101Error("Timeout reading message header")
            
            msg_id = struct.unpack('<H', header[0:2])[0]
            
            # Check if this is a long message
            if header[4] & 0x80:
                # Long message - read data length and payload
                data_len = struct.unpack('<H', header[2:4])[0]
                data = self.serial.read(data_len)
                if len(data) < data_len:
                    raise KIM101Error("Timeout reading message data")
            else:
                # Short message - no additional data
                data = b''
            
            if expected_msg_id is not None and msg_id != expected_msg_id:
                raise KIM101Error(
                    f"Received unexpected message ID: 0x{msg_id:04X}, "
                    f"expected 0x{expected_msg_id:04X}"
                )
            
            return msg_id, data
        
        finally:
            self.serial.timeout = old_timeout
    
    def _build_submsg_data(self, submsg_id: int, channel, 
                          payload: bytes) -> bytes:
        """Build sub-message data packet."""
        return struct.pack('<HH', submsg_id, channel) + payload
    
    # ========== Hardware Information Methods ==========
    
    async def get_hardware_info(self) -> dict[str, Any]:
        """
        Get hardware information from the controller.
        
        Returns:
            Dictionary with hardware information
        """
        self._send_message(self.MSG_HW_REQ_INFO)
        _, data = self._receive_message(self.MSG_HW_GET_INFO)
        
        if len(data) < 84:
            raise KIM101Error("Invalid hardware info response")
        
        serial_no = struct.unpack('<I', data[0:4])[0]
        model_no = data[4:12].decode('ascii').rstrip('\x00')
        hw_type = struct.unpack('<H', data[12:14])[0]
        fw_version = struct.unpack('<I', data[14:18])[0]
        hw_version = struct.unpack('<H', data[78:80])[0]
        mod_state = struct.unpack('<H', data[80:82])[0]
        num_channels = struct.unpack('<H', data[82:84])[0]
        
        return {
            'serial_number': serial_no,
            'model_number': model_no,
            'hardware_type': hw_type,
            'firmware_version': f"{(fw_version >> 24) & 0xFF}.{(fw_version >> 16) & 0xFF}.{fw_version & 0xFFFF}",
            'hardware_version': hw_version,
            'mod_state': mod_state,
            'num_channels': num_channels
        }
    
    # ========== Status Update Methods ==========
    
    async def start_status_updates(self) -> None:
        """Start automatic status update messages (10 Hz)."""
        self._send_short_message(self.MSG_HW_START_UPDATEMSGS, 0, 0)
        self._status_update_active = True
    
    async def stop_status_updates(self) -> None:
        """Stop automatic status update messages."""
        self._send_short_message(self.MSG_HW_STOP_UPDATEMSGS, 0, 0)
        self._status_update_active = False
    
    async def get_status_update(self, timeout: float = 2.0) -> List[List[int]]: #TODO Add automatic receiving for vals
        """
        Request and receive status update for all channels.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            List of ChannelStatus for each channel
        """
        self._send_short_message(self.MSG_PZMOT_REQ_STATUSUPDATE, 0, 0)
        _, data = self._receive_message(self.MSG_PZMOT_GET_STATUSUPDATE, timeout)
        
        if len(data) < 56:
            raise KIM101Error("Invalid status update response")
        
        statuses = []
        for i in range(4):
            offset = i * 14
            chan_ident = struct.unpack('<H', data[offset:offset+2])[0]
            position = struct.unpack('<l', data[offset+2:offset+6])[0]
            enc_count = struct.unpack('<l', data[offset+6:offset+10])[0]
            status_bits = struct.unpack('<I', data[offset+10:offset+14])[0]
            statuses.append([chan_ident, position, enc_count, status_bits])
        return statuses
    
    # ========== Position Control Methods ==========
    
    async def set_position_counter(self, channel: int, position: int = 0) -> None:
        """
        Set the position counter value (typically to zero).
        
        Args:
            channel: Channel to address
            position: Position value in steps (default: 0)
        """
        payload = struct.pack('<ll', position, 0)  # EncCount not used
        data = self._build_submsg_data(self.SUBMSG_PZMOT_POSCOUNTS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_position_counter(self, channel: int) -> int:
        """
        Get the current position counter value.
        
        Args:
            channel: Channel to query
            
        Returns:
            Position in steps
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_POSCOUNTS, channel)
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_POSCOUNTS:
            raise KIM101Error("Unexpected sub-message response")
        
        position = struct.unpack('<l', resp_data[4:8])[0]
        return position
    
    async def move_absolute(self, channel: int, position: int) -> None:
        """
        Move to an absolute position.
        
        Args:
            channel: Channel to move
            position: Target position in steps from zero
        """
        data = struct.pack('<Hl', channel, position)
        self._send_message(self.MSG_PZMOT_MOVE_ABSOLUTE, data)
    
    async def wait_for_move_complete(self, timeout: float = 30.0) -> Tuple[int, int]:
        """
        Wait for move completion message.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            Tuple of (channel, final_position)
        """
        _, data = self._receive_message(self.MSG_PZMOT_MOVE_COMPLETED, timeout)
        
        chan_ident = struct.unpack('<H', data[0:2])[0]
        position = struct.unpack('<l', data[2:6])[0]
        
        return chan_ident, position
    
    async def move_jog(self, channel: int, direction: int) -> None:
        """
        Execute a jog move.
        
        Args:
            channel (int): Channel to jog
            direction (int): 0x01 for 'forward' or 0x02 for 'reverse'
        """
        # jog_dir = 0x01 if direction.lower() == 'forward' else 0x02
        self._send_short_message(self.MSG_PZMOT_MOVE_JOG, channel, direction)

    
    # ========== Drive Parameter Methods ==========
    
    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> None:
        """
        Set drive operating parameters.
        
        Args:
            channel: Channel to configure
            params: DriveParameters object
        """
        if not (85 <= params[0] <= 125):
            raise ValueError("max_voltage must be between 85 and 125V")
        if not (1 <= params[1] <= 2000):
            raise ValueError("step_rate must be between 1 and 2000 steps/sec")
        if not (1 <= params[2] <= 100000):
            raise ValueError("step_accel must be between 1 and 100000 steps/sec^2")
        
        payload = struct.pack('<Hll', 
            params[0],
            params[1],
            params[2]
        )
        data = self._build_submsg_data(self.SUBMSG_PZMOT_DRIVEOP_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_drive_parameters(self, channel: int) -> DriveParameters:
        """
        Get drive operating parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            DriveParameters object
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_DRIVEOP_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_DRIVEOP_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        max_voltage, step_rate, step_accel = struct.unpack('<Hll', resp_data[4:14])
        
        return (max_voltage, step_rate, step_accel)
    
    # ========== Jog Parameter Methods ==========
    
    async def set_jog_parameters(self, channel: int, params: JogParameters) -> None:
        """
        Set jog parameters.
        
        Args:
            channel: Channel to configure
            params: JogParameters object
        """
        if not (1 <= params[1] <= 2000):
            raise ValueError("jog_step_size_fwd must be between 1 and 2000")
        if not (1 <= params[2] <= 2000):
            raise ValueError("jog_step_size_rev must be between 1 and 2000")
        if not (1 <= params[3] <= 2000):
            raise ValueError("jog_step_rate must be between 1 and 2000")
        if not (1 <= params[4] <= 100000):
            raise ValueError("jog_step_accel must be between 1 and 100000")
        
        payload = struct.pack('<Hllll',
            params[0], # jog_mode
            params[1], # jog_step_size_fwd
            params[2], # jog_step_size_rev
            params[3], # jog_step_rate
            params[4]  # jog_step_accel
        )
        
        data = self._build_submsg_data(self.SUBMSG_PZMOT_KCUBEJOG_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_jog_parameters(self, channel: int) -> JogParameters:
        """
        Get jog parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            JogParameters object
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_KCUBEJOG_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_KCUBEJOG_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        jog_mode, step_fwd, step_rev, rate, accel = struct.unpack('<Hllll', resp_data[4:24])
        
        return jog_mode, step_fwd, step_rev, rate, accel
    
    # ========== MMI (Joystick) Parameter Methods ==========
    
    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> None:
        """
        Set top panel joystick parameters.
        
        Args:
            channel: Channel to configure
            params: MMIParameters object
        """
        if not (1 <= params[1] <= 2000):
            raise ValueError("js_max_step_rate must be between 1 and 2000")
        if not (0 <= params[5] <= 100):
            raise ValueError("disp_brightness must be between 0 and 100")
        
        payload = struct.pack('<HlHllHH',
            params[0], # js_mode
            params[1], # js_max_step_rate
            params[2], # js_dir_sense
            params[3], # preset_pos1
            params[4], # preset_pos2
            params[5], # disp_brightness
            0
        )
        
        data = self._build_submsg_data(self.SUBMSG_PZMOT_KCUBEMMI_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_mmi_parameters(self, channel: int) -> MMIParameters:
        """
        Get top panel joystick parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            MMIParameters object
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_KCUBEMMI_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_KCUBEMMI_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        js_mode, max_rate, dir_sense, pos1, pos2, brightness = struct.unpack(
            '<HlHllH', resp_data[4:26])
        
        return js_mode, max_rate, dir_sense, pos1, pos2, brightness
    
    # ========== Trigger Configuration Methods ==========
    
    async def set_trigger_io_config(self, config: TriggerIOConfig) -> None:
        """
        Set trigger I/O configuration.
        
        Args:
            config: TriggerIOConfig object
        """
        payload = struct.pack('<HHHHHH',
            config[0], # trig_channel1
            config[1], # trig_channel2
            config[2], # trig1_mode
            config[3], # trig1_polarity
            config[4], # trig2_mode
            config[5]  # trig2_polarity
        ) + b'\x00' * 12  # Reserved bytes
        
        data = struct.pack('<H', self.SUBMSG_PZMOT_TRIGIO_CONFIG) + payload
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_trigger_io_config(self) -> TriggerIOConfig:
        """
        Get trigger I/O configuration.
        
        Returns:
            TriggerIOConfig object
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_TRIGIO_CONFIG, 0)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_TRIGIO_CONFIG:
            raise KIM101Error("Unexpected sub-message response")
        
        ch1, ch2, mode1, pol1, mode2, pol2 = struct.unpack('<HHHHHH', resp_data[2:14])
        
        return ch1, ch2, mode1, pol1, mode2, pol2
    
    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> None:
        """
        Set trigger position step parameters.
        
        Args:
            channel: Channel to configure
            params: TriggerParameters object
        """
        payload = struct.pack('<llllllll',
            params[0], # start_pos_fwd
            params[1], # interval_fwd
            params[2], # num_pulses_fwd
            params[3], # start_pos_rev
            params[4], # interval_rev
            params[5], # num_pulses_rev
            params[6], # pulse_width
            params[7]  # num_cycles
        )
        
        data = self._build_submsg_data(self.SUBMSG_PZMOT_TRIG_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_trigger_parameters(self, channel: int) -> TriggerParameters:
        """
        Get trigger position step parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            TriggerParameters object
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_TRIG_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_TRIG_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        values = struct.unpack('<llllllll', resp_data[4:36])
        
        return values
    
    # ========== Limit Switch Methods ==========
    
    async def set_limit_switch_params(self, channel: int, fwd_limit: int, rev_limit: int) -> None:
        """
        Set limit switch parameters.
        
        Args:
            channel: Channel to configure
            fwd_limit: Forward limit switch mode
            rev_limit: Reverse limit switch mode
        """
        payload = struct.pack('<HHH', fwd_limit, rev_limit, 0)
        
        data = self._build_submsg_data(self.SUBMSG_PZMOT_LIMSWITCH_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_limit_switch_params(self, channel: int) -> Tuple[int, int]:
        """
        Get limit switch parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            Tuple of (fwd_limit, rev_limit)
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_LIMSWITCH_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_LIMSWITCH_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        fwd, rev = struct.unpack('<HH', resp_data[4:8])
        
        return fwd, rev
    
    # ========== Feedback Signal Methods ==========
    
    async def set_feedback_signal_params(self, channel: int, mode: int) -> None:
        """
        Set feedback signal parameters.
        
        Args:
            channel: Channel to configure
            mode: Feedback signal mode
        """
        payload = struct.pack('<Hl', mode, 0)
        data = self._build_submsg_data(self.SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_feedback_signal_params(self, channel: int) -> int:
        """
        Get feedback signal parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            Tuple of (mode, encoder_const)
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        mode, _ = struct.unpack('<Hl', resp_data[4:10])
        
        return mode
    
    # ========== Channel Enable Methods ==========
    
    async def set_channel_enable_mode(self, mode: int) -> None:
        """
        Set channel enable mode.
        
        Args:
            mode: Channel enable mode
        """
        data = struct.pack('<HH', self.SUBMSG_PZMOT_CHANENABLE_MODE, mode)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_channel_enable_mode(self) -> int:
        """
        Get channel enable mode.
        
        Returns:
            ChannelEnableMode
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_CHANENABLE_MODE, 0)
        
        msg_id, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_CHANENABLE_MODE:
            raise KIM101Error("Unexpected sub-message response")
        
        mode = struct.unpack('<H', resp_data[2:4])[0]
        
        return mode
    
    # ========== Move Parameters for Trigger Methods ==========
    
    async def set_move_relative_params(self, channel: int, distance: int) -> None:
        """
        Set relative move distance for trigger input.
        
        Args:
            channel: Channel to configure
            distance: Relative distance in steps (can be negative)
        """
        payload = struct.pack('<l', distance)
        data = self._build_submsg_data(self.SUBMSG_PZMOT_MOVERELATIVE_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_move_relative_params(self, channel: int) -> int:
        """
        Get relative move distance for trigger input.
        
        Args:
            channel: Channel to query
            
        Returns:
            Relative distance in steps
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_MOVERELATIVE_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_MOVERELATIVE_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        distance = struct.unpack('<l', resp_data[4:8])[0]
        return distance
    
    async def set_move_absolute_params(self, channel: int, position: int) -> None:
        """
        Set absolute move position for trigger input.
        
        Args:
            channel: Channel to configure
            distance: Absolute position in steps
        """
        payload = struct.pack('<l', position)
        data = self._build_submsg_data(self.SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS, channel, payload)
        self._send_message(self.MSG_PZMOT_SET_PARAMS, data)
    
    async def get_move_absolute_params(self, channel: int) -> int:
        """
        Get absolute move position for trigger input.
        
        Args:
            channel: Channel to query
            
        Returns:
            Absolute position in steps
        """
        self._send_short_message(self.MSG_PZMOT_REQ_PARAMS, self.SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS, channel)
        
        _, resp_data = self._receive_message(self.MSG_PZMOT_GET_PARAMS)
        
        submsg_id = struct.unpack('<H', resp_data[0:2])[0]
        if submsg_id != self.SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS:
            raise KIM101Error("Unexpected sub-message response")
        
        distance = struct.unpack('<l', resp_data[4:8])[0]
        return distance