import serial
import struct
from enum import IntEnum
from typing import Optional, Union
from kim101_driver_I import *

class MSG_ID(IntEnum):
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

class SUBMSG_ID(IntEnum):
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

class KIM101(KIM101Interface):
    """
    Thorlabs KIM101 K-Cube Inertial Piezo Motor Controller Driver
    
    This class provides a comprehensive interface for controlling the KIM101
    controller via serial communication.
    """
    
    # Device identifiers
    DEST_GENERIC_USB = 0x50
    SOURCE_GENERIC_USB = 0x01

    # TODO rework all I/O values to simple types, which can be parsed with PYON
    
    def __init__(self, device: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize KIM101 controller connection.
        
        Args:
            device: Serial device name
            baudrate: Communication baud rate (default: 115200)
            timeout: Serial timeout in seconds (default: 1.0)
        """
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._status_update_active = False
        self.connect()
    
    def log(self, msg: str, cmd: Optional[str] = None) -> None:
        log = f"[{self.__class__.__name__}]"
        if cmd is not None:
            log += f" <CMD: {cmd}>"
        log += f" {msg}"
        print(log)
        
    def connect(self) -> COMMAND_STATUS:
        """Open serial connection to the controller."""
        try:
            self.serial = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
        except serial.SerialException as e:
            self.log(f"Failed to connect to {self.device}")
            return COMMAND_STATUS.COM_ERROR
        return COMMAND_STATUS.OK
    
    def close(self) -> None:
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self._send_short_message(MSG_ID.MSG_HW_STOP_UPDATEMSGS, 0, 0)
            self._status_update_active = False
            self.serial.close()
    
    def _send_message(self, msg_id: MSG_ID, data: bytes,
                     dest: int = DEST_GENERIC_USB, 
                     source: int = SOURCE_GENERIC_USB) -> COMMAND_STATUS:
        """
        Send a message to the controller.
        
        Args:
            msg_id: Message ID (2 bytes)
            data: Data payload
            dest: Destination byte
            source: Source byte
        """
        if not self.serial or not self.serial.is_open:
            if self.connect() == COMMAND_STATUS.COM_ERROR:
                return COMMAND_STATUS.COM_ERROR

        data_len = len(data)
        header = struct.pack('<HHBB', msg_id.value, data_len, dest | 0x80, source)
        msg = header + data
        
        written = self.serial.write(msg)
        if written != len(msg):
            self.log(f"<CMD {msg_id.name}> Number of written bytes [{written}] is different from expected [{len(msg)}]")
            return COMMAND_STATUS.COM_ERROR
        return COMMAND_STATUS.OK
    
    def _send_short_message(self, msg_id: MSG_ID, param1: int, param2: int,
                            dest: int = DEST_GENERIC_USB,
                            source: int = SOURCE_GENERIC_USB) -> COMMAND_STATUS:
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
            if self.connect() == COMMAND_STATUS.COM_ERROR:
                return COMMAND_STATUS.COM_ERROR
    
        msg = struct.pack('<HBBBB', msg_id.value, param1, param2, dest, source)
        written = self.serial.write(msg)
        if written != len(msg):
            self.log(f"Number of written bytes [{written}] is different from expected [{len(msg)}]",
                      msg_id.name)
            return COMMAND_STATUS.COM_ERROR
        return COMMAND_STATUS.OK
    
    def _read_simple_param(self, submsg_id: SUBMSG_ID, channel: int) -> Union[COMMAND_STATUS, bytes]:
        """Read parameter values using a MSG_PZMOT_REQ_PARAMS command

        Args:
            submsg_id: ID of the parameter to be read
            channel: 
        """
        resp = self._send_short_message(MSG_ID.MSG_PZMOT_REQ_PARAMS, submsg_id.value, channel)
        if resp != COMMAND_STATUS.OK:
            return resp
        
        resp = self._receive_message(MSG_ID.MSG_PZMOT_GET_PARAMS)
        if type(resp) is COMMAND_STATUS:
            return resp
        data = resp[1]
        read_submsg_id = struct.unpack('<H', data[0:2])[0]
        if read_submsg_id != submsg_id:
            return COMMAND_STATUS.SUBMSG_ERROR
        
        return data
    
    def _receive_message(self, expected_msg_id: Optional[MSG_ID] = None) -> Union[COMMAND_STATUS, tuple[int, bytes]]:
        """
        Receive a message from the controller.
        
        Args:
            expected_msg_id: Expected message ID (or None to accept any)
            timeout: Override default timeout
            
        Returns:
            Tuple of (message_id, data)
        """
        if not self.serial or not self.serial.is_open:
            if self.connect() == COMMAND_STATUS.COM_ERROR:
                return COMMAND_STATUS.COM_ERROR
            # self.serial: serial.Serial

        try:
            # Read header (6 bytes minimum)
            msg_name = expected_msg_id.name if expected_msg_id is not None else None
            header = self.serial.read(6)
            if len(header) < 6:
                self.log("Timeout reading message header", msg_name)
                return COMMAND_STATUS.COM_ERROR
            
            msg_id = struct.unpack('<H', header[0:2])[0]
            
            # Check if this is a long message
            if header[4] & 0x80:
                # Long message - read data length and payload
                data_len = struct.unpack('<H', header[2:4])[0]
                data = self.serial.read(data_len)
                if len(data) < data_len:
                    self.log("Timeout reading message data", msg_name)
                    return COMMAND_STATUS.COM_ERROR
            else:
                # Short message - no additional data
                data = b''
            
            if expected_msg_id is not None and MSG_ID(msg_id) != expected_msg_id:
                self.log(
                    f"Received unexpected message ID: 0x{msg_id:04X}, "
                    f"expected 0x{expected_msg_id.value:04X}", msg_name
                )
        except:
            return COMMAND_STATUS.COM_ERROR
        
        return msg_id, data
    
    def _build_submsg_data(self, submsg_id: SUBMSG_ID, channel, 
                          payload: bytes) -> bytes:
        """Build sub-message data packet."""
        return struct.pack('<HH', submsg_id.value, channel) + payload
    
    # ========== Hardware Information Methods ==========
    
    async def get_hardware_info(self) -> Union[str, dict[str, Union[int, str]]]:
        """
        Get hardware information from the controller.
        
        Returns:
            Dictionary with hardware information
        """
        resp = self._send_short_message(MSG_ID.MSG_HW_REQ_INFO, 0, 0)
        if resp != COMMAND_STATUS.OK:
            return resp.value
        
        resp = self._receive_message(MSG_ID.MSG_HW_GET_INFO)

        if type(resp) == COMMAND_STATUS:
            return resp.value
        
        data = resp[1]
        if len(data) < 84:
            self.log("Invalid hardware info response", MSG_ID.MSG_HW_GET_INFO.name)
            return COMMAND_STATUS.DATA_ERROR.value
        
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
    
    async def start_status_updates(self) -> str:
        result = self._send_short_message(MSG_ID.MSG_HW_START_UPDATEMSGS, 0, 0)
        if result == COMMAND_STATUS.OK:
            self._status_update_active = True
        return result.value 
    
    async def stop_status_updates(self) -> str:
        result = self._send_short_message(MSG_ID.MSG_HW_STOP_UPDATEMSGS, 0, 0)
        if result == COMMAND_STATUS.OK:
            self._status_update_active = False
        return result.value 
    
    async def get_status_update(self, timeout: float = 2.0) -> Union[str, list[list[int]]]:
        resp = self._send_short_message(MSG_ID.MSG_PZMOT_REQ_STATUSUPDATE, 0, 0)
        if resp != COMMAND_STATUS.OK:
            return resp.value

        old_timeout = self.serial.timeout
        self.serial.timeout = timeout
        resp = self._receive_message(MSG_ID.MSG_PZMOT_GET_STATUSUPDATE)
        self.serial.timeout = old_timeout
        
        if type(resp) == COMMAND_STATUS:
            return resp.value
        
        data = resp[1]
        if len(data) < 56:
            self.log(f"<CMD: {MSG_ID.MSG_PZMOT_GET_STATUSUPDATE.name}> Invalid status update response")
            return COMMAND_STATUS.COM_ERROR.value
        
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
    
    async def set_position_counter(self, channel: int, position: int = 0) -> str:
        payload = struct.pack('<ll', position, 0)  # EncCount not used
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_POSCOUNTS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_position_counter(self, channel: int) -> Union[str, int]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_POSCOUNTS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        position = struct.unpack('<l', data[4:8])[0]
        return position
    
    async def move_absolute(self, channel: int, position: int) -> str:
        data = struct.pack('<Hl', channel, position)
        return self._send_message(MSG_ID.MSG_PZMOT_MOVE_ABSOLUTE, data).value
    
    async def wait_for_move_complete(self, timeout: float = 30.0) -> Union[str, tuple[int, int]]:
        old_timeout = self.serial.timeout
        self.serial.timeout = timeout
        resp = self._receive_message(MSG_ID.MSG_PZMOT_MOVE_COMPLETED)
        self.serial.timeout = old_timeout
        if type(resp) == COMMAND_STATUS:
            return resp.value
        data = resp[1]
        
        chan_ident = struct.unpack('<H', data[0:2])[0]
        position = struct.unpack('<l', data[2:6])[0]
        
        return chan_ident, position
    
    async def move_jog(self, channel: int, direction: int) -> str:
        return self._send_short_message(MSG_ID.MSG_PZMOT_MOVE_JOG, channel, direction).value

    
    # ========== Drive Parameter Methods ==========
    
    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> str:
        
        if not (85 <= params[0] <= 125):
            self.log(f"<CMD: {SUBMSG_ID.SUBMSG_PZMOT_DRIVEOP_PARAMS.name}> max_voltage must be between 85 and 125V")
            return COMMAND_STATUS.VALUE_ERROR.value
        if not (1 <= params[1] <= 2000):
            self.log(f"<CMD: {SUBMSG_ID.SUBMSG_PZMOT_DRIVEOP_PARAMS.name}> step_rate must be between 1 and 2000 steps/sec")
            return COMMAND_STATUS.VALUE_ERROR.value
        if not (1 <= params[2] <= 100000):
            self.log(f"<CMD: {SUBMSG_ID.SUBMSG_PZMOT_DRIVEOP_PARAMS.name}> step_accel must be between 1 and 100000 steps/sec^2")
            return COMMAND_STATUS.VALUE_ERROR.value
        
        payload = struct.pack('<Hll', 
            params[0],
            params[1],
            params[2]
        )
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_DRIVEOP_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_drive_parameters(self, channel: int) -> Union[str, DriveParameters]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_DRIVEOP_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        max_voltage, step_rate, step_accel = struct.unpack('<Hll', data[4:14])
        return (max_voltage, step_rate, step_accel)
    
    # ========== Jog Parameter Methods ==========
    
    async def set_jog_parameters(self, channel: int, params: JogParameters) -> str:
        
        if not (1 <= params[1] <= 2000):
            self.log("jog_step_size_fwd must be between 1 and 2000")
            return COMMAND_STATUS.VALUE_ERROR.value
        if not (1 <= params[2] <= 2000):
            self.log("jog_step_size_rev must be between 1 and 2000")
            return COMMAND_STATUS.VALUE_ERROR.value
        if not (1 <= params[3] <= 2000):
            self.log("jog_step_rate must be between 1 and 2000")
            return COMMAND_STATUS.VALUE_ERROR.value
        if not (1 <= params[4] <= 100000):
            self.log("jog_step_accel must be between 1 and 100000")
            return COMMAND_STATUS.VALUE_ERROR.value
        
        payload = struct.pack('<Hllll',
            params[0], # jog_mode
            params[1], # jog_step_size_fwd
            params[2], # jog_step_size_rev
            params[3], # jog_step_rate
            params[4]  # jog_step_accel
        )
        
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_KCUBEJOG_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_jog_parameters(self, channel: int) -> Union[str, JogParameters]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_KCUBEJOG_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        jog_mode, step_fwd, step_rev, rate, accel = struct.unpack('<Hllll', data[4:24])
        return jog_mode, step_fwd, step_rev, rate, accel
    
    # ========== MMI (Joystick) Parameter Methods ==========
    
    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> str:
        
        if not (1 <= params[1] <= 2000):
            self.log("js_max_step_rate must be between 1 and 2000")
            return COMMAND_STATUS.VALUE_ERROR.value 
        if not (0 <= params[5] <= 100):
            self.log("disp_brightness must be between 0 and 100")
            return COMMAND_STATUS.VALUE_ERROR.value
        
        payload = struct.pack('<HlHllHH',
            params[0], # js_mode
            params[1], # js_max_step_rate
            params[2], # js_dir_sense
            params[3], # preset_pos1
            params[4], # preset_pos2
            params[5], # disp_brightness
            0
        )
        
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_KCUBEMMI_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_mmi_parameters(self, channel: int) -> Union[str, MMIParameters]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_KCUBEMMI_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        js_mode, max_rate, dir_sense, pos1, pos2, brightness = struct.unpack(
            '<HlHllH', data[4:26])
        
        return js_mode, max_rate, dir_sense, pos1, pos2, brightness
    
    # ========== Trigger Configuration Methods ==========
    
    async def set_trigger_io_config(self, config: TriggerIOConfig) -> str:
        payload = struct.pack('<HHHHHH',
            config[0], # trig_channel1
            config[1], # trig_channel2
            config[2], # trig1_mode
            config[3], # trig1_polarity
            config[4], # trig2_mode
            config[5]  # trig2_polarity
        ) + b'\x00' * 12  # Reserved bytes
        
        data = struct.pack('<H', SUBMSG_ID.SUBMSG_PZMOT_TRIGIO_CONFIG.value) + payload #TODO
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_trigger_io_config(self) -> Union[str, TriggerIOConfig]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_TRIGIO_CONFIG, 0)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        ch1, ch2, mode1, pol1, mode2, pol2 = struct.unpack('<HHHHHH', data[2:14])
        
        return ch1, ch2, mode1, pol1, mode2, pol2
    
    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> str: 
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
        
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_TRIG_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_trigger_parameters(self, channel: int) -> Union[str, TriggerParameters]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_TRIG_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        values = struct.unpack('<llllllll', data[4:36])
        
        return values
    
    # ========== Limit Switch Methods ==========
    
    async def set_limit_switch_params(self, channel: int, params: LimitSwitchParameters) -> str:
        payload = struct.pack('<HHH', params[0], params[1], 0)
        
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_LIMSWITCH_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_limit_switch_params(self, channel: int) -> Union[str, LimitSwitchParameters]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_LIMSWITCH_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        fwd, rev = struct.unpack('<HH', data[4:8])
        
        return fwd, rev
    
    # ========== Feedback Signal Methods ==========
    
    async def set_feedback_signal_params(self, channel: int, mode: int) -> str:
        payload = struct.pack('<Hl', mode, 0)
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_feedback_signal_params(self, channel: int) -> Union[str, int]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_FEEDBACK_SIG_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        mode = struct.unpack('<H', data[4:6])[0]
        
        return mode
    
    # ========== Channel Enable Methods ==========
    
    async def set_channel_enable_mode(self, mode: int) -> str:
        data = struct.pack('<HH', SUBMSG_ID.SUBMSG_PZMOT_CHANENABLE_MODE.value, mode)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_channel_enable_mode(self) -> Union[str, int]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_CHANENABLE_MODE, 0)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        mode = struct.unpack('<H', data[2:4])[0]
        
        return mode
    
    # ========== Move Parameters for Trigger Methods ==========
    
    async def set_move_relative_params(self, channel: int, distance: int) -> str:
        payload = struct.pack('<l', distance)
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_MOVERELATIVE_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_move_relative_params(self, channel: int) -> Union[int, str]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_MOVERELATIVE_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        distance = struct.unpack('<l', data[4:8])[0]
        return distance
    
    async def set_move_absolute_params(self, channel: int, position: int) -> str:
        payload = struct.pack('<l', position)
        data = self._build_submsg_data(SUBMSG_ID.SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS, channel, payload)
        return self._send_message(MSG_ID.MSG_PZMOT_SET_PARAMS, data).value
    
    async def get_move_absolute_params(self, channel: int) -> Union[int, str]:
        data = self._read_simple_param(SUBMSG_ID.SUBMSG_PZMOT_MOVEABSOLUTE_PARAMS, channel)
        if type(data) == COMMAND_STATUS:
            return data.value
        
        position = struct.unpack('<l', data[4:8])[0]
        return position