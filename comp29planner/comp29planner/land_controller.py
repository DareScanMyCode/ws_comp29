import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Point, TwistStamped
from comp29msg.msg import DetectionResult
import rclpy.time
from .utils import *
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
    def __init__(self, args=None):
        self.dx = 0.
        self.dy = 0.
        self.Quadrant = 0

    def land_ctrl(self, dx, dy):    
        self.ready_flag = False
        Quadrant=self.Quadrant
        # 如果收到消息且未过时
        if self.ready_flag:
            if dx>0.1 or dy>0.1:    
                # 象限
                kx=2.
                ky=2.
                if Quadrant==2:
                    vel_set_frd_x =  ky*dy
                    vel_set_frd_y = -kx*dx
                    vel_set_frd_z =  0.0
                elif Quadrant==1:
                    vel_set_frd_x =  ky*dy
                    vel_set_frd_y =  kx*dx
                    vel_set_frd_z =  0.0
                elif Quadrant==3:
                    vel_set_frd_x =  -ky*dy
                    vel_set_frd_y =  -kx*dx
                    vel_set_frd_z =  0.0
                elif Quadrant==4:         
                    vel_set_frd_x =  -ky*dy
                    vel_set_frd_y =  kx*dx
                    vel_set_frd_z =  0.0
            else:
                vel_set_frd_x =  0
                vel_set_frd_y =  0
                vel_set_frd_z =  0.15
        else:
            vel_set_frd_x =  0.0
            vel_set_frd_y =  0.0
            vel_set_frd_z =  0.05

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
