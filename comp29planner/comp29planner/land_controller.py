import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Point, TwistStamped
from comp29msg.msg import DetectionResult
import rclpy.time
from .utils import *
from typing import Literal
MAX_LATENCY = rclpy.time.Duration(seconds=1,nanoseconds=0)

# 判断在哪个象限（图是左上角为原点 →为X正方向 ↓为Y正方向）
def JudgeQuadrant(axis_x,axis_y):
    distanceX=axis_x-0.5
    distanceY=axis_y-0.5
    if (distanceX<0 and distanceY<0):
        return 2
    if (distanceX>0 and distanceY<0):
        return 1
    if (distanceX<0 and distanceY>0):
        return 3
    if (distanceX>0 and distanceY>0):
        return 4

    
class LandController():
    def __init__(self, land_pos:Literal['left_top', 'right_top', 'bottom', 'central']='central', k_img=0.5, land_spd = 0.15, threshold=0.1):
        self.k_img = k_img  
        self.land_spd = land_spd
        self.threshold = threshold
        self.final_land_flag = False
        self.dx = 0.
        self.dy = 0.
        self.Quadrant = 0
        self.land_pos = land_pos
        self.min_height = 1
        
        # TODO 根据land_pos来设定停机位置
        if self.land_pos == 'left_top':
            self.tgt_num_pos_img = [0.75, 0.75]
        elif self.land_pos == 'right_top':
            self.tgt_num_pos_img = [0.25, 0.75]
        elif self.land_pos == 'bottom':
            self.tgt_num_pos_img = [0.5, 0.25]
        elif self.land_pos == 'central':
            self.tgt_num_pos_img = [0.5, 0.5]
        else:
            self.tgt_num_pos_img = [0.5, 0.5]

    def land_ctrl(self, num_x, num_y, height):    
        # -------------------------------- X
        # |
        # |
        # |
        # | Y
        # x_err 为正，说明目标在左边，速度向左
        # y_err 为正，说明目标在前边，速度向前
        # 对准就降
        if abs(num_x - self.tgt_num_pos_img[0]) < self.threshold and abs(num_y - self.tgt_num_pos_img[1]) < self.threshold:
            # self.final_land_flag = True
            print("已经对准目标")
            return [0.0, 0.0, self.land_spd]
        # 没对准但高度很低也降
        if height <= self.min_height:
            return [0.0, 0.0, self.land_spd]
        # 否则调整位置
        x_err = self.tgt_num_pos_img[0] - num_x
        y_err = self.tgt_num_pos_img[1] - num_y
        
        vel_set_frd_x = self.k_img * x_err
        vel_set_frd_y = self.k_img * y_err
        vel_set_frd_z = 0.0

        return [vel_set_frd_x, vel_set_frd_y, vel_set_frd_z]


class LandControllerNode(Node):
    def __init__(self, args=None):
        super().__init__('land_controller')
        self.publisher = self.create_publisher(TwistStamped, '/vel_set_frd', 10)
        detect_re_sub = self.create_subscription(  
            DetectionResult,  
            'detected_pose_topic',  
            self.detected_pose_call_back,  
            10)
        self.land_ctrler = LandController()
        self.create_timer(0.1, self.land)
        self.get_logger().info(f">>> {ColorCodes.blue}LandController node has started. {ColorCodes.reset}")
        
    def detected_pose_call_back(self, msg):
        msg_t = msg.header.stamp
        if self.get_clock().now() - rclpy.time.Time.from_msg(msg_t) < MAX_LATENCY:
            self.ready_flag = True
            pos = msg.position
            x_mid = pos.x
            y_mid = pos.y
            self.land_ctrler.Quadrant=JudgeQuadrant(x_mid, y_mid)
            self.land_ctrler.dx=abs(x_mid-0.5)
            self.land_ctrler.dy=abs(y_mid-0.5)

    def land(self):
        vv = self.land_ctrler.land_ctrl(self.land_ctrler.dx, self.land_ctrler.dy)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = vv[0]
        vel_msg.twist.linear.y = vv[1]
        vel_msg.twist.linear.z = vv[2]
        self.publisher.publish(vel_msg)
        
    def run(self):
        rclpy.spin(self)
        rclpy.shutdown()

 

def main(args=None):
    rclpy.init(args=args)
    ctrl = LandController()
    try:
        rclpy.spin(ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.destroy_node()
        rclpy.shutdown()

ctrl = LandController()
