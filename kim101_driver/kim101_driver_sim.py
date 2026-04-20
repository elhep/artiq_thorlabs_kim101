from kim101_driver_I import *

class KIM101Sim(KIM101Interface):

    def __init__(self):
        self.status_updates = False
        self.zero_position = [0] * 4
        self.abs_position = [0] * 4
        self.last_ch_move = 1
        
        self.jog_params = [JogParameters((1, 1, 1, 1, 1))] * 4
        self.drv_params = [DriveParameters((85, 1, 1))] * 4
        self.trig_params = [TriggerParameters((1, 1, 1, 1, 1, 1, 1, 1))] * 4
        self.trig_io_params = TriggerIOConfig((1, 1, 0, 1, 0, 1))
        self.mmi_params = [MMIParameters((1, 1, 0, 1, 1, 100))] * 4
        self.limsw_params = [LimitSwitchParameters((1, 1))] * 4
        self.feedback_mode = [0] * 4
        self.enable_mode = 0
        self.rel_dist = [0] * 4
        self.abs_move = [0] * 4


    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    async def get_hardware_info(self) -> Union[str, dict[str, Union[int, str]]]:
        return {
            'serial_number': 94000009,
            'model_number': "ABCDEF12",
            'hardware_type': 44,
            'firmware_version': 0x01020300,
            'hardware_version': 0x2222,
            'mod_state': 0x1111,
            'num_channels': 4
        }

    async def start_status_updates(self) -> str:
        print(f"Simulated: Enable status updates")
        self.status_updates = True
        return COMMAND_STATUS.OK.value

    async def stop_status_updates(self) -> str:
        print(f"Simulated: Disable status updates")
        self.status_updates = False
        return COMMAND_STATUS.OK.value

    def __get_ch_idx(self, ch_val: int) -> int:
        for i in range(4):
            if (ch_val >> i) & 0x01:
                return i + 1
        raise ValueError(f"Invalid channel value [{ch_val}]")

    async def get_status_update(self, timeout: float = 2.0) -> Union[str, list[list[int]]]:
        pass

    async def set_position_counter(self, channel: int, position: int = 0) -> str:
        self.zero_position[self.__get_ch_idx(channel)] = position
        return COMMAND_STATUS.OK.value

    async def get_position_counter(self, channel: int) -> Union[int, str]:
        return self.zero_position[self.__get_ch_idx(channel)]

    async def move_absolute(self, channel: int, position: int) -> str:
        self.last_ch_move = self.__get_ch_idx(channel)
        self.abs_position[self.last_ch_move] = position
        return COMMAND_STATUS.OK.value

    async def wait_for_move_complete(self, timeout: float = 30.0) -> Union[str, tuple[int, int]]:
        return self.last_ch_move, self.abs_position[self.last_ch_move]

    async def move_jog(self, channel: int, direction: int) -> str:
        ch = self.__get_ch_idx(channel)
        if direction == 0x01:
            self.abs_position[ch] += self.jog_params[ch][1]
        elif direction == 0x02:
            self.abs_position[ch] -= self.jog_params[ch][2]
        else:
            print(f"Unknown direction value [{direction}]")
            return COMMAND_STATUS.VALUE_ERROR.value
        self.last_ch_move = ch
        return COMMAND_STATUS.OK.value


    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> str:
        if not (85 <= params[0] <= 125):
            raise ValueError("max_voltage must be between 85 and 125V")
        if not (1 <= params[1] <= 2000):
            raise ValueError("step_rate must be between 1 and 2000 steps/sec")
        if not (1 <= params[2] <= 100000):
            raise ValueError("step_accel must be between 1 and 100000 steps/sec^2")
        self.drv_params[self.__get_ch_idx(channel)] = params
        return COMMAND_STATUS.OK.value

    async def get_drive_parameters(self, channel: int) -> Union[str, DriveParameters]:
        return self.drv_params[self.__get_ch_idx(channel)]


    async def set_jog_parameters(self, channel: int, params: JogParameters) -> str:
        if not (1 <= params[1] <= 2000):
            raise ValueError("jog_step_size_fwd must be between 1 and 2000")
        if not (1 <= params[2] <= 2000):
            raise ValueError("jog_step_size_rev must be between 1 and 2000")
        if not (1 <= params[3] <= 2000):
            raise ValueError("jog_step_rate must be between 1 and 2000")
        if not (1 <= params[4] <= 100000):
            raise ValueError("jog_step_accel must be between 1 and 100000")
        self.jog_params[self.__get_ch_idx(channel)] = params
        return COMMAND_STATUS.OK.value

    async def get_jog_parameters(self, channel: int) -> Union[str, JogParameters]:
        return self.jog_params[self.__get_ch_idx(channel)]


    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> str:
        if not (1 <= params[1] <= 2000):
            raise ValueError("js_max_step_rate must be between 1 and 2000")
        if not (0 <= params[5] <= 100):
            raise ValueError("disp_brightness must be between 0 and 100")
        self.mmi_params[self.__get_ch_idx(channel)] = params
        return COMMAND_STATUS.OK.value

    async def get_mmi_parameters(self, channel: int) -> Union[str, MMIParameters]:
        return self.mmi_params[self.__get_ch_idx(channel)]


    async def set_trigger_io_config(self, config: TriggerIOConfig) -> str:
        self.trig_io_params = config
        return COMMAND_STATUS.OK.value

    async def get_trigger_io_config(self) -> Union[str, TriggerIOConfig]:
        return self.trig_io_params

    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> str:
        self.trig_params[self.__get_ch_idx(channel)] = params
        return COMMAND_STATUS.OK.value

    async def get_trigger_parameters(self, channel: int) -> Union[str, TriggerParameters]:
        return self.trig_params[self.__get_ch_idx(channel)]


    async def set_limit_switch_params(self, channel: int, params: LimitSwitchParameters) -> str:
        self.limsw_params[self.__get_ch_idx(channel)] = params
        return COMMAND_STATUS.OK.value

    async def get_limit_switch_params(self, channel: int) -> Union[str, LimitSwitchParameters]:
        return self.limsw_params[self.__get_ch_idx(channel)]


    async def set_feedback_signal_params(self, channel: int, mode: int) -> str:
        self.feedback_mode[self.__get_ch_idx(channel)] = mode
        return COMMAND_STATUS.OK.value

    async def get_feedback_signal_params(self, channel: int) -> Union[str, int]:
        return self.feedback_mode[self.__get_ch_idx(channel)]


    async def set_channel_enable_mode(self, mode: int) -> str:
        self.enable_mode = mode
        return COMMAND_STATUS.OK.value

    async def get_channel_enable_mode(self) -> Union[str, int]:
        return self.enable_mode


    async def set_move_relative_params(self, channel: int, distance: int) -> str:
        self.rel_dist[self.__get_ch_idx(channel)] = distance
        return COMMAND_STATUS.OK.value

    async def get_move_relative_params(self, channel: int) -> Union[str, int]:
        return self.rel_dist[self.__get_ch_idx(channel)]

    async def set_move_absolute_params(self, channel: int, position: int) -> str:
        self.abs_move[self.__get_ch_idx(channel)] = position
        return COMMAND_STATUS.OK.value

    async def get_move_absolute_params(self, channel: int) -> Union[str, int]:
        return self.abs_move[self.__get_ch_idx(channel)]