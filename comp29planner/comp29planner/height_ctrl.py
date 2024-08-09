

class HightCtrl:
    def __init__(self, tgt_height, k_h=0.1) -> None:
        self.tgt_height = tgt_height
        self.k_h = k_h
        pass
    
    def get_ctrl(self, height):
        return self.k_h * (self.tgt_height - height)
    
    
class YawCtrl:
    def __init__(self, tgt_yaw_angle, k_yaw=0.1) -> None:
        self.tgt_yaw_angel = tgt_yaw_angle
        self.k_yaw = k_yaw
        
    def get_ctrl(self, yaw_angle):
        # TODO 角度问题！
        return self.k_yaw * (self.tgt_yaw_angel - yaw_angle)