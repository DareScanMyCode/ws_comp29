import numpy as np

class HightCtrl:
    def __init__(self, tgt_height, k_p=0.4, k_i=0.02) -> None:
        self.tgt_height = tgt_height
        self.k_p = k_p
        self.k_i = k_i
        self.pre_height = 0
        pass
    
    def get_ctrl(self, height, rpy):
        if np.linalg.norm(rpy)!=0:#没收到imu数据
            #我的测距是跟着机体走的，所以要加上机体的横滚、俯仰角
            height = height*np.cos(rpy[2])*np.cos(rpy[1])
            
        if self.pre_height == 0:
            vz = self.k_p * (self.tgt_height - height)
        else:
            vz = self.k_p * (self.tgt_height - height) + self.k_i * (self.tgt_height - height - self.pre_height)
        self.pre_height = height
        if vz<-0.3:
            vz=-0.3
        if vz>0.3:
            vz=0.3
        return -vz
    
class YawCtrl:
    def __init__(self, tgt_yaw_angle, k_yaw=0.1) -> None:
        self.tgt_yaw_angel = tgt_yaw_angle
        self.k_yaw = k_yaw
        
    def get_ctrl(self, yaw_angle):
        # TODO 角度问题！
        return self.k_yaw * (self.tgt_yaw_angel - yaw_angle)