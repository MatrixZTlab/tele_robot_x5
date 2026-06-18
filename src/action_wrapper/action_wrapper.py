from src.get_stream import PiperStream
from src.get_stream import OculusReaderStream
from scipy.spatial.transform import Rotation
from src.config import RobConfig
from src.config import TeleDeviceConfig
from src.config import (left_piper2right_rob,right_piper2left_rob,right_piper2indus_rob,
                        piper_scale)
from src.config import (quest2left_rob,quest2right_rob,quest_scale)

import numpy as np

class ActionWrapper:
    def __init__(self, tele_device:str, device_name:str):
        if 'use_quest' == tele_device:
            if 'ctrl_left' == device_name:
                tele_config = TeleDeviceConfig(device_name,quest2right_rob,quest_scale)
                rob_config = RobConfig()
            elif 'ctrl_right' == device_name:
                tele_config = TeleDeviceConfig(device_name,quest2left_rob,quest_scale)
                rob_config = RobConfig()
            else:
                raise ValueError(f"输入的名称和摇操设备不匹配")
            self.tele_config = tele_config
            self.stream = OculusReaderStream(self.tele_config.device_name)
        elif 'use_piper' == tele_device:
            if 'can_left'== device_name:
                tele_config = TeleDeviceConfig(device_name,left_piper2right_rob,piper_scale)
                rob_config = RobConfig()
            elif 'can_right' == device_name:
                tele_config = TeleDeviceConfig(device_name,right_piper2indus_rob,piper_scale)
                rob_config = RobConfig()
            else:
                raise ValueError(f"输入的名称和摇操设备不匹配")
            self.tele_config = tele_config
            self.stream = PiperStream(self.tele_config.device_name)
        else:
            raise ValueError(f"输入摇操设备错误")
        self.control_active = False
        self.safe_flag = True
        self.rob_config = rob_config
        self.tele_reference_pose = np.eye(4)
        self.tele_pos_cmp_pre = np.zeros(3)
        self.tele_rot_cmp_pre = np.eye(3)
        self.rob_reference_pos = np.zeros(3)
        self.rob_reference_rot = np.eye(3)
    def run_recv(self):
        self.stream.run()
    def stop_recv(self):
        self.stream.stop()
    def get_action(self,obs):
        rob_current_pose = obs["rob_pose_rpy"]
        rob_current_state = obs["rob_work_state"]
        rob_handle = obs["rob_handle"]
        rob_current_pos = rob_current_pose[:3]
        rob_current_rpy = rob_current_pose[3:]
        rob_current_rot = Rotation.from_euler(seq='xyz', angles=rob_current_rpy, degrees=True).as_matrix()

        pose_data,trigger_data= self.stream.get_data()
        if len(pose_data) == 0 or trigger_data is None:
            print("没有收到数据，查看piper连接")
            return {
                'command_pose':rob_current_pose,
                'trigger':0
            }
        trigger = trigger_data
        # 遥操触发
        if trigger:
            # 已完成触发后的初始化
            if self.control_active:
                tele_current_pose = pose_data
                tele_delta_rot = tele_current_pose[:3, :3] @ np.linalg.inv(self.tele_reference_pose[:3, :3])
                tele_delta_pos = tele_current_pose[:3, 3] - self.tele_reference_pose[:3, 3]
                tele_pos_cmp = tele_delta_pos + self.tele_pos_cmp_pre
                tele_rot_cmp = tele_delta_rot @ self.tele_rot_cmp_pre
                rob_delta_pos = self.tele_config.device2rob[:3, :3] @ (tele_pos_cmp * self.tele_config.trans_scale)
                rob_delta_rot = self.tele_config.device2rob[:3, :3] @ tele_rot_cmp @ self.tele_config.rob2device[:3, :3]
                rob_next_rot =  rob_delta_rot @ self.rob_reference_rot
                # rob_next_rot = self.rob_reference_rot
                rob_next_rpy = Rotation.from_matrix(matrix=rob_next_rot).as_euler("xyz",degrees=True)
                rob_next_pos = rob_delta_pos + self.rob_reference_pos
                rob_next_pose = np.concatenate([rob_next_pos, rob_next_rpy])
                if self.safe_flag:
                    if not self.rob_config.judge_safe_joint(rob_handle,rob_next_pose) or \
                        not self.rob_config.judge_safe_zone(rob_next_pos):
                        tele_pos_cmp = self.tele_pos_cmp_pre
                        rob_delta_pos = self.tele_config.device2rob[:3, :3] @ (tele_pos_cmp * self.tele_config.trans_scale)
                        rob_next_pos = rob_delta_pos + self.rob_reference_pos
                        tele_rot_cmp = self.tele_rot_cmp_pre
                        rob_delta_rot = self.tele_config.device2rob[:3,:3] @ tele_rot_cmp @ self.tele_config.rob2device[:3, :3]
                        rob_next_rot = rob_delta_rot @ self.rob_reference_rot
                        rob_next_rpy = Rotation.from_matrix(matrix=rob_next_rot).as_euler("xyz", degrees=True)
                        rob_next_pose = np.concatenate([rob_next_pos, rob_next_rpy])
                self.tele_pos_cmp_pre = tele_pos_cmp
                self.tele_rot_cmp_pre = tele_rot_cmp
                self.tele_reference_pose = tele_current_pose
                return {
                    'command_pose': rob_next_pose,
                    'trigger': trigger
                }
            else:  # last state is not in active
                if rob_current_state == "control":
                    self.control_active = True
                    self.tele_reference_pose = pose_data
                    self.rob_reference_rot = rob_current_rot
                    self.rob_reference_pos = rob_current_pos
                    self.tele_pos_cmp_pre = np.zeros(3)
                    self.tele_rot_cmp_pre = np.eye(3)
                    if self.safe_flag:
                        self.rob_config.set_sphere_center(rob_handle)
                return {
                    'command_pose': rob_current_pose,
                    'trigger': trigger
                }
        else:
            self.control_active = False
            self.tele_reference_pose = np.eye(4)
            self.rob_reference_rot = rob_current_rot
            self.rob_reference_pos = rob_current_pos
            self.tele_pos_cmp_pre = np.zeros(3)
            self.tele_rot_cmp_pre = np.eye(3)
            return {
                'command_pose': rob_current_pose,
                'trigger': trigger
            }



