from kim101_driver_I import *

class KIM101Sim(KIM101Interface):

    def __init__(self):
        self.status_updates = False
        self.zero_position = [0] * 4
        self.abs_position = [0] * 4
        self.last_ch_move = 1
        
        self.jog_params = [JogParameters((1, 1, 1, 1, 1))] * 4
        self.drv_params = [DriveParameters((85, 1, 1))] * 4
        self.trig_params = [TriggerParameters(1, 1, 1, 1, 1, 1, 1, 1)] * 4
        self.trig_io_params = TriggerIOConfig(1, 1, 0, 1, 0, 1)
        self.mmi_params = [MMIParameters(1, 1, 0, 1, 1, 100)] * 4
        self.limsw_params = [LimitSwitchParamers(1, 1)] * 4
        self.feedback_mode = [0] * 4
        self.enable_mode = 0
        self.rel_dist = [0] * 4
        self.abs_move = [0] * 4


    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    async def get_hardware_info(self) -> dict[str, Any]:
        return {
            'serial_number': 94000009,
            'model_number': "ABCDEF12",
            'hardware_type': 44,
            'firmware_version': 0x01020300,
            'hardware_version': 0x2222,
            'mod_state': 0x1111,
            'num_channels': 4
        }

    async def start_status_updates(self) -> None:
        logging.warning(f"Simulated: Enable status updates")
        self.status_updates = True

    async def stop_status_updates(self) -> None:
        logging.warning(f"Simulated: Disable status updates")
        self.status_updates = False

    def __get_ch_idx(self, ch_val: int) -> int:
        for i in range(4):
            if (ch_val >> i) & 0x01:
                return i + 1
        raise ValueError(f"Invalid channel value [{ch_val}]")

    async def get_status_update(self, timeout: float = 2.0) -> List[List[int]]:
        pass

    async def set_position_counter(self, channel: int, position: int = 0) -> None:
        self.zero_position[self.__get_ch_idx(channel)] = position

    async def get_position_counter(self, channel: int) -> int:
        return self.zero_position[self.__get_ch_idx(channel)]

    async def move_absolute(self, channel: int, position: int) -> None:
        self.last_ch_move = self.__get_ch_idx(channel)
        self.abs_position[self.last_ch_move] = position

    async def wait_for_move_complete(self, timeout: float = 30.0) -> Tuple[int, int]:
        return self.last_ch_move, self.abs_position[self.last_ch_move]

    async def move_jog(self, channel: int, direction: int) -> None:
        ch = self.__get_ch_idx(channel)
        if direction == 0x01:
            self.abs_position[ch] += self.jog_params[ch][1]
        elif direction == 0x02:
            self.abs_position[ch] -= self.jog_params[ch][2]
        else:
            raise ValueError(f"Unknown direction value [{direction}]")
        self.last_ch_move = ch


    async def set_drive_parameters(self, channel: int, params: DriveParameters) -> None:
        if not (85 <= params[0] <= 125):
            raise ValueError("max_voltage must be between 85 and 125V")
        if not (1 <= params[1] <= 2000):
            raise ValueError("step_rate must be between 1 and 2000 steps/sec")
        if not (1 <= params[2] <= 100000):
            raise ValueError("step_accel must be between 1 and 100000 steps/sec^2")
        self.drv_params[self.__get_ch_idx(channel)] = params

    async def get_drive_parameters(self, channel: int) -> DriveParameters:
        self.drv_params[self.__get_ch_idx(channel)]


    async def set_jog_parameters(self, channel: int, params: JogParameters) -> None:
        if not (1 <= params[1] <= 2000):
            raise ValueError("jog_step_size_fwd must be between 1 and 2000")
        if not (1 <= params[2] <= 2000):
            raise ValueError("jog_step_size_rev must be between 1 and 2000")
        if not (1 <= params[3] <= 2000):
            raise ValueError("jog_step_rate must be between 1 and 2000")
        if not (1 <= params[4] <= 100000):
            raise ValueError("jog_step_accel must be between 1 and 100000")
        self.jog_params[self.__get_ch_idx(channel)] = params

    async def get_jog_parameters(self, channel: int) -> JogParameters:
        return self.jog_params[self.__get_ch_idx(channel)]


    async def set_mmi_parameters(self, channel: int, params: MMIParameters) -> None:
        if not (1 <= params[1] <= 2000):
            raise ValueError("js_max_step_rate must be between 1 and 2000")
        if not (0 <= params[5] <= 100):
            raise ValueError("disp_brightness must be between 0 and 100")
        self.mmi_params[self.__get_ch_idx(channel)] = params

    async def get_mmi_parameters(self, channel: int) -> MMIParameters:
        return self.mmi_params[self.__get_ch_idx(channel)]


    async def set_trigger_io_config(self, config: TriggerIOConfig) -> None:
        self.trig_io_params = config

    async def get_trigger_io_config(self) -> TriggerIOConfig:
        return self.trig_io_params

    async def set_trigger_parameters(self, channel: int, params: TriggerParameters) -> None:
        self.trig_params[self.__get_ch_idx(channel)] = params

    async def get_trigger_parameters(self, channel: int) -> TriggerParameters:
        return self.trig_params[self.__get_ch_idx(channel)]


    async def set_limit_switch_params(self, channel: int, params: LimitSwitchParamers) -> None:
        self.limsw_params[self.__get_ch_idx(channel)] = params

    async def get_limit_switch_params(self, channel: int) -> LimitSwitchParamers:
        return self.limsw_params[self.__get_ch_idx(channel)]


    async def set_feedback_signal_params(self, channel: int, mode: int) -> None:
        self.feedback_mode[self.__get_ch_idx[channel]] = mode

    async def get_feedback_signal_params(self, channel: int) -> int:
        return self.feedback_mode[self.__get_ch_idx[channel]]


    async def set_channel_enable_mode(self, mode: int) -> None:
        self.enable_mode = mode

    async def get_channel_enable_mode(self) -> int:
        return self.enable_mode


    async def set_move_relative_params(self, channel: int, distance: int) -> None:
        self.rel_dist[self.__get_ch_idx(channel)] = distance

    async def get_move_relative_params(self, channel: int) -> int:
        return self.rel_dist[self.__get_ch_idx(channel)]

    async def set_move_absolute_params(self, channel: int, position: int) -> None:
        self.abs_move[self.__get_ch_idx(channel)] = position

    async def get_move_absolute_params(self, channel: int) -> int:
        return self.abs_move[self.__get_ch_idx(channel)]