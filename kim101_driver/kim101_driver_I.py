import abc
from typing import Union
from enum import IntEnum, Enum

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

class KIM101Exception(Exception):
    pass

DriveParameters = tuple[int, int, int]
'''Parameters of the DriveOPParameters subcommands
* [0] - MaxVoltage: Maximum voltage of the driver in range 85-125 V
* [1] - StepRate: Speed of the voltage ramp in steps/sec (range: 1-2000)
* [2] - StepAccel: Acceleration of the voltage ramp in steps/sec^2 (range: 1-100 000)
'''

JogParameters = tuple[int, int, int, int, int]
'''Parameters of the PZMOT_KCubeJogParams subcommands
* [0] - JogMode - Mode of jog movement
    * 0x01: Continuous - After receiving jog signal driver continuously generates ramps using JogStepRate and JogStepAccn parameters
    * 0x02: Single - After receiving jog signal driver performs one step defined by JogStepSizeFwd and JogStepSizeRev parameters
* [1] - JogStepSizeFwd - Number of steps in one forward jog in steps (range: 1-2000)
* [2] - JogStepSizeRev - Number of steps in one reverse jog in steps (range: 1-2000)
* [3] - JogStepRate - Speed of the voltage ramp in steps/sec (range: 1-2000)
* [4] - JogStepAccn: Acceleration of the step rate in steps/sec^2 (range: 1-100 000)
'''

MMIParameters = tuple[int, int, int, int, int, int]
'''Parameters of the PZMOT_KCubeMMIParams subcommands
* [0] - JSMode: Mode of joystick operation. Available values:
    * 0x01: Velocity control mode - movement caused by joystick is proportional to its deflection 
    and limited by JSMaxStepRate
    * 0x02: Jog mode - Joystick deflection causes jog move
    * 0x03: Position mode - Joystick deflection left or up causes chnnels to move to preset 1; 
    deflection right or down - move to preset 2
* [1] - JSMaxStepRate: Maximal speed of the joystick movement in steps/sec (range: 1-2000)
* [2] - JSDirSense: Direction of movement triggered by joystick. Avaliable values:
    * 0x00: Joystick can't trigger movement
    * 0x01: Up/Right joystick causes forward motion
    * 0x02: Up/Right joystick causes reverse motion
* [3] - PresetPos1: Preset 1 used in the position mode
* [4] - PresetPos2: Preset 2 used in the position mode
* [5] - DispBrightness: Brightness of the LCD display (range: 1-100)
'''

TriggerIOConfig = tuple[int, int, int, int, int, int]
'''Parameters of the PZMOT_KCubeTrigIOConfig subcommands
* [0] - TrigChannel1: drive channel using IO1 as trigger. Accepted values:
    * 0x01: CH1
    * 0x02: CH2
    * 0x04: CH3
    * 0x08: CH4
* [1] - TrigChannel2: drive channel using IO2 as trigger. Accepted values:
    * 0x01: CH1
    * 0x02: CH2
    * 0x04: CH3
    * 0x08: CH4
* [2] - Trig1Mode - mode of the IO1 trigger. Accepted values:
    * 0x00: DISABLED
    * 0x01: GPI
    * 0x02: RELMOVE
    * 0x03: ABSMOVE
    * 0x04: RESETCOUNT
    * 0x0A: GPO
    * 0x0B: INMOTION
    * 0x0C: MAXVELOCITY
    * 0x0D: POSSTEPS_FWD
    * 0x0E: POSSTEPS_REV
    * 0x0F: POSSTEPS_BOTH
    * 0x10: FWDLIMIT
    * 0x11: REVLIMIT
    * 0x12: EITHERLIMIT
* [3] - Trig1Polarity - polarity of the IO1 trigger. Accepted values:
    * 0x01: HIGH
    * 0x02: LOW
* [4] - Trig2Mode - mode of the IO1 trigger. Accepted values:
    * 0x00: DISABLED
    * 0x01: GPI
    * 0x02: RELMOVE
    * 0x03: ABSMOVE
    * 0x04: RESETCOUNT
    * 0x0A: GPO
    * 0x0B: INMOTION
    * 0x0C: MAXVELOCITY
    * 0x0D: POSSTEPS_FWD
    * 0x0E: POSSTEPS_REV
    * 0x0F: POSSTEPS_BOTH
    * 0x10: FWDLIMIT
    * 0x11: REVLIMIT
    * 0x12: EITHERLIMIT
* [5] - Trig2Polarity - polarity of the IO2 trigger. Accepted values:
    * 0x01: HIGH
    * 0x02: LOW
'''

TriggerParameters = tuple[int, int, int, int, int, int, int, int]
'''Parameters of the PZMOT_KCubeTrigParams subcommands
* [0] - StartPosFwd: position (position steps) in a forward move, at which pulse sequence generation starts
* [1] - IntervalFwd: position interval (position steps) between pulses in a forward move
* [2] - NumPulsesFwd: number of pulses in a sequence in a forward move
* [3] - StartPosRev: position (position steps) in a reverse move, at which pulse sequence generation starts
* [4] - IntervalRev: position interval (position steps) between pulses in a reverse move
* [5] - NumPulsesRev: number of pulses in a sequence in a reverse move
* [6] - PulseWidth: duration of pulses (1 - 100 000 us)
* [7] - NumCycles: number of forward/reverse pulse sequences
'''

LimitSwitchParameters = tuple[int, int]
'''Parameters of Get_PZMOT_LimSwitchParams subcommands
* [0] - FwdHardLimit
* [1] - RevHardLimit
'''

class COMMAND_STATUS(Enum):
    OK = ":OK"
    ERROR = ":ERR"

    SUBMSG_ERROR = ":ERR:SUBMSG"
    DATA_ERROR = ":ERR:DATA"
    COM_ERROR = ":ERR:COM"
    VALUE_ERROR = ":ERR:VAL"

class KIM101Interface:

    @abc.abstractmethod
    async def get_hardware_info(self) -> Union[str, dict[str, Union[int, str]]]:
        '''Request and receive hardware info about the device
        
        '''
        pass

    @abc.abstractmethod
    async def start_status_updates(self) -> str:
        """Start automatic status update messages (10 Hz).
        
        Returns:
            Operation status
        """

    @abc.abstractmethod
    async def stop_status_updates(self) -> str:
        """Stop automatic status update messages.
        
        Returns:
            Operation status
        """

    @abc.abstractmethod
    async def get_status_update(self, timeout: float = 2.0) -> Union[str, list[list[int]]]:
        """
        Request and receive status update for all channels.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            List of ChannelStatus for each channel
        """

    @abc.abstractmethod
    async def set_position_counter(self, channel: int, position: int = 0) -> str:
        """
        Set the position counter value (typically to zero).
        
        Args:
            channel: Channel to address
            position: Position value in steps (default: 0)
        """

    @abc.abstractmethod
    async def get_position_counter(self, channel: int) -> Union[str, int]:
        """
        Get the current position counter value.
        
        Args:
            channel: Channel to query
            
        Returns:
            Position in steps
        """

    @abc.abstractmethod
    async def move_absolute(self, channel: int, position: int) -> str:
        """
        Move to an absolute position.
        
        Args:
            channel: Channel to move
            position: Target position in steps from zero
        """

    @abc.abstractmethod
    async def wait_for_move_complete(self, timeout: float = 30.0) -> Union[str, tuple[int, int]]:
        """
        Wait for move completion message.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            Tuple of (channel, final_position)
        """

    @abc.abstractmethod
    async def move_jog(self, channel: int, direction: int) -> str:
        """
        Execute a jog move.
        
        Args:
            channel (int): Channel to jog
            direction (int): 0x01 for 'forward' or 0x02 for 'reverse'
        """


    @abc.abstractmethod
    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> str:
        """
        Set drive operating parameters.
        
        Args:
            channel: Channel to configure
            params: DriveParameters object
        """

    @abc.abstractmethod
    async def get_drive_parameters(self, channel: int) -> Union[str, DriveParameters]:
        """
        Get drive operating parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            DriveParameters object
        """


    @abc.abstractmethod
    async def set_jog_parameters(self, channel: int, params: JogParameters) -> str:
        """
        Set jog parameters.
        
        Args:
            channel: Channel to configure
            params: JogParameters object
        """

    @abc.abstractmethod
    async def get_jog_parameters(self, channel: int) -> Union[str, JogParameters]:
        """
        Get jog parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            JogParameters object
        """


    @abc.abstractmethod
    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> str:
        """
        Set top panel joystick parameters.
        
        Args:
            channel: Channel to configure
            params: MMIParameters object
        """

    @abc.abstractmethod
    async def get_mmi_parameters(self, channel: int) -> Union[str, MMIParameters]:
        """
        Get top panel joystick parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            MMIParameters object
        """


    @abc.abstractmethod
    async def set_trigger_io_config(self, config: TriggerIOConfig) -> str:
        """
        Set trigger I/O configuration.
        
        Args:
            config: TriggerIOConfig object
        """

    @abc.abstractmethod
    async def get_trigger_io_config(self) -> Union[str, TriggerIOConfig]:
        """
        Get trigger I/O configuration.
        
        Returns:
            TriggerIOConfig object
        """

    @abc.abstractmethod
    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> str:
        """
        Set trigger position step parameters.
        
        Args:
            channel: Channel to configure
            params: TriggerParameters object
        """

    @abc.abstractmethod
    async def get_trigger_parameters(self, channel: int) -> Union[str, TriggerParameters]:
        """
        Get trigger position step parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            TriggerParameters object
        """


    @abc.abstractmethod
    async def set_limit_switch_params(self, channel: int, params: LimitSwitchParameters) -> str:
        """
        Set limit switch parameters.
        
        Args:
            channel: Channel to configure
            fwd_limit: Forward limit switch mode
            rev_limit: Reverse limit switch mode
        """

    @abc.abstractmethod
    async def get_limit_switch_params(self, channel: int) -> Union[str, LimitSwitchParameters]:
        """
        Get limit switch parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            Tuple of (fwd_limit, rev_limit)
        """


    @abc.abstractmethod
    async def set_feedback_signal_params(self, channel: int, mode: int) -> str:
        """
        Set feedback signal parameters.
        
        Args:
            channel: Channel to configure
            mode: Feedback signal mode
        """

    @abc.abstractmethod
    async def get_feedback_signal_params(self, channel: int) -> Union[str, int]:
        """
        Get feedback signal parameters.
        
        Args:
            channel: Channel to query
            
        Returns:
            mode
        """


    @abc.abstractmethod
    async def set_channel_enable_mode(self, mode: int) -> str:
        """
        Set channel enable mode.
        
        Args:
            mode: Channel enable mode
        """

    @abc.abstractmethod
    async def get_channel_enable_mode(self) -> Union[int, str]:
        """
        Get channel enable mode.
        
        Returns:
            ChannelEnableMode
        """


    @abc.abstractmethod
    async def set_move_relative_params(self, channel: int, distance: int) -> str:
        """
        Set relative move distance for trigger input.
        
        Args:
            channel: Channel to configure
            distance: Relative distance in steps (can be negative)
        """

    @abc.abstractmethod
    async def get_move_relative_params(self, channel: int) -> Union[int, str]:
        """
        Get relative move distance for trigger input.
        
        Args:
            channel: Channel to query
            
        Returns:
            Relative distance in steps
        """

    @abc.abstractmethod
    async def set_move_absolute_params(self, channel: int, position: int) -> str:
        """
        Set absolute move position for trigger input.
        
        Args:
            channel: Channel to configure
            distance: Absolute position in steps
        """

    @abc.abstractmethod
    async def get_move_absolute_params(self, channel: int) -> Union[int, str]:
        """
        Get absolute move position for trigger input.
        
        Args:
            channel: Channel to query
            
        Returns:
            Absolute position in steps
        """
