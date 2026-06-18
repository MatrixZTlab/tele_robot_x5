import xapi.api as xad
import numpy as np
from src.action_wrapper import ActionWrapper

import time
import threading

class MultiControl:
    def __init__(self, ip:str = "192.168.1.6", tele_device:str = "use_quest", device_name:str = "ctrl_right"):
        self.ip = ip
        self.action = ActionWrapper(tele_device, device_name=device_name)
        self.current_state = "deactive"
        self.trigger = False
        self.process_on = False
        self.thread = None
        self.handle = None

    def start_process(self):
        self.action.run_recv()
        self.handle = xad.connect(self.ip)
        xad.reset(self.handle)
        time.sleep(0.5)
        if xad.get_system_state(self.handle).alarm:
            raise ValueError(f"机器人{self.which_hand}报警未消除")
        time.sleep(0.5)
        xad.set_remote(self.handle, False)
        if xad.get_system_state(self.handle).remote:
            raise ValueError(f"机器人{self.which_hand}处于远程")
        xad.set_system_mode(self.handle, 100)
        time.sleep(0.5)
        if xad.get_system_state(self.handle).mode != 100:
            raise ValueError(f"机器人{self.which_hand}没有位于自动CMD模式")

        self.process_on = True
        self.thread = threading.Thread(target=self.run_process)
        self.thread.start()

    def stop_process(self):
        self.action.stop_recv()
        self.process_on = False
        self.thread.join()
        xad.disconnect(self.handle)
        print("Process stopped")

    def get_position(self, handle):
        cpoint = xad.get_cpoint(handle)
        return cpoint

    def get_joint(self, handle):
        jpoint = xad.get_cjoint(handle)
        return jpoint

    def get_observations(self):

        return (
            {"rob_pose_rpy": np.array(self.get_position(self.handle).pose.tolist())[:6],
             "rob_work_state": self.current_state,
             "rob_handle": self.handle}
        )

    def state_transition(self):

        match self.current_state:
            case "deactive":
                if self.trigger:
                    self.current_state = "active"
            case "active":
                if not self.trigger:
                    self.current_state = "deactive"
                else:
                    xad.enable_servo(self.handle, True)
                    time.sleep(0.05)
                    if xad.get_system_state(self.handle).enable:
                        self.action.act_drag()
                        self.current_state = "control"
            case "control":
                if not self.trigger:
                    xad.enable_servo(self.handle, False)
                    if not xad.get_system_state(self.handle).enable:
                        self.action.deact_drag()
                        self.current_state = "deactive"
                else:
                    xad.rgm_start(self.handle)
                    self.current_state = "moving"
            case "moving":
                if not self.trigger:
                    xad.rgm_stop(self.handle)
                    xad.abort(self.handle)
                    if xad.get_system_state(self.handle).in_pos:
                        xad.enable_servo(self.handle, False)
                        if not xad.get_system_state(self.handle).enable:
                            self.action.deact_drag()
                            self.current_state = "deactive"

    
    def servo_motion(self, pose_cmd):
        match self.current_state:
            case "deactive":
                pass
            case "active":
                pass
            case "control":
                pass
            case "moving":
                xad.servol(self.handle, pose_cmd, 
                           0, 0, 10, 100, 100)
        return 0

    def run_process(self):
        cycle_time = 0.005
        print("Process started")
        while self.process_on:
            time.sleep(cycle_time)
            point_cmd = self.get_position(self.handle)
            obs = self.get_observations()
            action = self.action.get_action(obs)
            point_cmd.pose.x = action["command_pose"][0]
            point_cmd.pose.y = action["command_pose"][1]
            point_cmd.pose.z = action["command_pose"][2]
            point_cmd.pose.a = action["command_pose"][3]
            point_cmd.pose.b = action["command_pose"][4]
            point_cmd.pose.c = action["command_pose"][5]
            self.trigger = action["trigger"]
            self.state_transition()
            # self.servo_motion(point_cmd.pose)



if __name__ == "__main__":

    left_multi_control = MultiControl()
    left_multi_control.start_process()
