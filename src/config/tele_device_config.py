import numpy as np
right_piper2left_rob = np.array(
    [[-1,  0,  0,  0],
     [ 0,  0, -1,  0],
     [ 0, -1,  0,  0],
     [ 0,  0,  0,  1]]
)
left_piper2right_rob = np.array(
    [[-1,  0,  0,  0],
     [ 0,  0,  1,  0],
     [ 0,  1,  0,  0],
     [ 0,  0,  0,  1]]
)
piper_scale = np.array([1,1,1])
right_piper2indus_rob = np.array(
    [[-1,  0,  0,  0],
     [ 0, -1,  0,  0],
     [ 0,  0,  1,  0],
     [ 0,  0,  0,  1]
    ]
)
quest2left_rob = np.array(
    [[ 0,  0, -1,  0],
     [ 0, -1,  0,  0],
     [-1,  0,  0,  0],
     [ 0,  0,  0,  1]]
)
quest_scale = np.array([1000,1000,1000])
quest2right_rob = np.array(
    [[ 0,  0, -1,  0],
     [ 0,  1,  0,  0],
     [ 1,  0,  0,  0],
     [ 0,  0,  0,  1]]
)
class TeleDeviceConfig:
    def __init__(self,device_name:str,device2rob:np.ndarray,trans_scale:np.ndarray):
        self.device_name = device_name
        self.device2rob = device2rob
        self.trans_scale = trans_scale
    @property
    def device2rob(self):
        return self._device2rob
    @device2rob.setter
    def device2rob(self,value):
        if not np.linalg.det(value):
            raise ValueError(f"输入的转换矩阵奇异")
        self._device2rob = value

    @property
    def rob2device(self):
        return np.linalg.inv(self.device2rob)

    @property
    def trans_scale(self):
        return self._trans_scale
    @trans_scale.setter
    def trans_scale(self,value):
        if not(len(value)==3):
            raise ValueError(f"输入尺度个数应为3")
        self._trans_scale = value



