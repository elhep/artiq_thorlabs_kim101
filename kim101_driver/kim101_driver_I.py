import abc
from typing import Union

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

LimitSwitchParamers = tuple[int, int]
'''Parameters of Get_PZMOT_LimSwitchParams subcommands
* [0] - FwdHardLimit
* [1] - RevHardLimit
'''

class KIM101Interface:

    @abc.abstractmethod
    async def get_hardware_info(self) -> dict[str, Any]:
        pass

    @abc.abstractmethod
    async def start_status_updates(self) -> None:
        pass

    @abc.abstractmethod
    async def stop_status_updates(self) -> None:
        pass

    @abc.abstractmethod
    async def get_status_update(self, timeout: float = 2.0) -> list[list[int]]:
        pass

    @abc.abstractmethod
    async def set_position_counter(self, channel: int, position: int = 0) -> None:
        pass

    @abc.abstractmethod
    async def get_position_counter(self, channel: int) -> int:
        pass

    @abc.abstractmethod
    async def move_absolute(self, channel: int, position: int) -> None:
        pass

    @abc.abstractmethod
    async def wait_for_move_complete(self, timeout: float = 30.0) -> tuple[int, int]:
        pass

    @abc.abstractmethod
    async def move_jog(self, channel: int, direction: int) -> None:
        pass


    @abc.abstractmethod
    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> None:
        pass

    @abc.abstractmethod
    async def get_drive_parameters(self, channel: int) -> DriveParameters:
        pass


    @abc.abstractmethod
    async def set_jog_parameters(self, channel: int, params: JogParameters) -> None:
        pass

    @abc.abstractmethod
    async def get_jog_parameters(self, channel: int) -> JogParameters:
        pass


    @abc.abstractmethod
    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> None:
        pass

    @abc.abstractmethod
    async def get_mmi_parameters(self, channel: int) -> MMIParameters:
        pass


    @abc.abstractmethod
    async def set_trigger_io_config(self, config: TriggerIOConfig) -> None:
        pass

    @abc.abstractmethod
    async def get_trigger_io_config(self) -> TriggerIOConfig:
        pass

    @abc.abstractmethod
    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> None:
        pass

    @abc.abstractmethod
    async def get_trigger_parameters(self, channel: int) -> TriggerParameters:
        pass


    @abc.abstractmethod
    async def set_limit_switch_params(self, channel: int, fwd_limit: int, rev_limit: int) -> None:
        pass

    @abc.abstractmethod
    async def get_limit_switch_params(self, channel: int) -> tuple[int, int]:
        pass


    @abc.abstractmethod
    async def set_feedback_signal_params(self, channel: int, mode: int, encoder_const: int = 0) -> None:
        pass

    @abc.abstractmethod
    async def get_feedback_signal_params(self, channel: int) -> tuple[int, int]:
        pass


    @abc.abstractmethod
    async def set_channel_enable_mode(self, mode: int) -> None:
        pass

    @abc.abstractmethod
    async def get_channel_enable_mode(self) -> int:
        pass


    @abc.abstractmethod
    async def set_move_relative_params(self, channel: int, distance: int) -> None:
        pass

    @abc.abstractmethod
    async def get_move_relative_params(self, channel: int) -> int:
        pass

    @abc.abstractmethod
    async def set_move_absolute_params(self, channel: int, distance: int) -> None:
        pass

    @abc.abstractmethod
    async def get_move_absolute_params(self, channel: int) -> int:
        pass