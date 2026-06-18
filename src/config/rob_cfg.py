import numpy as np
import xapi.api as xad
class RobConfig:
    JOINT_RANGE_DEFAULT = {'j1': {'min': -120, 'max': 120}, 'j2': {'min': -70, 'max': 100}, 'j3': {'min': -180, 'max': 50},
                           'j4': {'min': -160, 'max': 160}, 'j5': {'min': -100, 'max': 100}, 'j6': {'min': -180, 'max': 180},
                           'e1': {'min': -360, 'max': 360}, 'e2': {'min': -360, 'max': 360}, 'e3': {'min': -360, 'max': 360}}
    SPHERE_RADIUS_DEFAULT = 300.0
    SPHERE_CENTER_DEFAULT = xad.Pose()

    def __init__(self, center=None, radius=None, range=None):
        if center is None:
            center = self.SPHERE_CENTER_DEFAULT
        if radius is None:
            radius = self.SPHERE_RADIUS_DEFAULT
        if range is None:
            range = self.JOINT_RANGE_DEFAULT

        self.sphere_radius = radius
        self.sphere_center = center
        self.joint_range = range

    def judge_safe_zone(self,pos_cmd=np.zeros(3)):
        if ((pos_cmd[0]-self.sphere_center.x)**2
            + (pos_cmd[1]-self.sphere_center.y)**2
            + (pos_cmd[2]-self.sphere_center.z)**2)\
            >= self.sphere_radius**2:
            return False
        return True
    def set_sphere_center(self,handle):
        self.sphere_center = xad.get_cpoint(handle).pose

    def judge_safe_joint(self, handle, point_cmd=np.zeros(6)):
        joint_fb = xad.get_cjoint(handle)
        point = xad.get_cpoint(handle)
        point.pose.x = point_cmd[0]
        point.pose.y = point_cmd[1]
        point.pose.z = point_cmd[2]
        point.pose.a = point_cmd[3]
        point.pose.b = point_cmd[4]
        point.pose.c = point_cmd[5]
        joint_cmd = xad.cnvrt_j(handle,point,2,joint_fb).tolist()
        for (num, range),joint in zip(self.joint_range.items(), joint_cmd):
            if joint <= range['min'] or joint>= range['max']:
                return False
        return True





