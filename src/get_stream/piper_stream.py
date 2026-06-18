from piper_sdk import  *
from threading import Event, Lock, Thread
from scipy.spatial.transform import Rotation
import time
import numpy as np

class PiperStream:
    def __init__(self,can_name:str="can0"):
        self.__stream_thread = None
        self.__stream_lock = Lock()
        self.__stop_stream_event = Event()
        self._piper = C_PiperInterface_V2(can_name)
        self._piper_end_pose = self._piper.ArmEndPose()
        self._piper_gripper = self._piper.ArmGripper()

    @staticmethod
    def _is_connect_ok(piper:C_PiperInterface_V2):
        connect_flag = piper.isOk()
        if not connect_flag:
            print("机械臂没有连接")
        return connect_flag

    @staticmethod
    def _is_enable_ok(piper:C_PiperInterface_V2):
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        while not enable_flag:
            elapsed_time = time.time() - start_time
            enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status \
                      and piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status \
                      and piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status \
                      and piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status \
                      and piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status \
                      and piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            if not enable_flag:
                piper.EnableArm(7,enable_flag=0x02)
                piper.GripperCtrl(0, 1000, 0x01, 0XAE)
            else:
                break
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                enable_flag = False
                print("机械臂没有上使能")
                break
            time.sleep(1)
        return enable_flag

    def run(self):
        self._piper.ConnectPort(True)
        if self._is_connect_ok(self._piper):
            self._piper.EnableArm(7,enable_flag=0x02)
            self._piper.GripperCtrl(0, 1000, 0x01, 0xAE)
            if self._is_enable_ok(self._piper):
                self.__stop_stream_event.clear()
                if not self.__stream_thread or not self.__stream_thread.alive():
                    try:
                        self.__stream_thread = Thread(target=self._update_data, daemon=True)
                        self.__stream_thread.start()
                    except Exception:
                        self.__stop_stream_event.set()

    def get_data(self):
        end_pos = np.zeros(6)
        end_pose_mat = np.eye(4)
        with self.__stream_lock:
            end_pos[0] = round(self._piper_end_pose.end_pose.X_axis * 0.001,3)
            end_pos[1]= round(self._piper_end_pose.end_pose.Y_axis * 0.001,3)
            end_pos[2] = round(self._piper_end_pose.end_pose.Z_axis * 0.001,3)
            end_pos[3] = round(self._piper_end_pose.end_pose.RX_axis * 0.001,3)
            end_pos[4] = round(self._piper_end_pose.end_pose.RY_axis * 0.001,3)
            end_pos[5] = round(self._piper_end_pose.end_pose.RZ_axis * 0.001,3)
            gripper_angle = self._piper_gripper.gripper_state.grippers_angle
        end_pose_mat[:3,:3] = Rotation.from_euler(seq='xyz',angles=end_pos[3:],degrees=True).as_matrix()
        end_pose_mat[:3,3] = end_pos[:3]
        return end_pose_mat, gripper_angle>30000

    def stop(self):
        self.__stop_stream_event.set()
        if hasattr(self,'__stream_thread') and self.__stream_thread is not None and self.__stream_thread.alive():
            self.__stream_thread.join()
    def act_drag(self):
        self._piper.MotionCtrl_1(0x00,0x00,0x01)
    def deact_drag(self):
        self._piper.MotionCtrl_1(0x00, 0x00, 0x02)
    def _update_data(self) -> None:
        while not self.__stop_stream_event.is_set():
            time.sleep(0.005)
            with self.__stream_lock:
                self._piper_end_pose = self._piper.GetArmEndPoseMsgs()
                # print("end:",self._piper_end_pose.end_pose)
                self._piper_gripper = self._piper.GetArmGripperMsgs()


if __name__ == "__main__":
    data_stream = PiperStream(can_name='can_right')
    data_stream.run()
    time_pre = time.time()
    try:
        while True:
            time.sleep(0.005)
            time_delay = time.time()-time_pre
            end_pose, grip = data_stream.get_data()
            print('grip:',grip)
            if time_delay > 250:
                break
    except KeyboardInterrupt:
        pass
    finally:
        data_stream.stop()





















