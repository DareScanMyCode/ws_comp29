# 核心规划器
# 核心规划器
# from .gport_sdk.goprt import GportGimbal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from quaternions import Quaternion as Quaternion
import math
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
import numpy as np
from scipy.optimize import minimize
from utils import *
from sensor_msgs.msg        import Imu
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import Float32MultiArray
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped
from std_msgs.msg           import Int64, Float64, Bool
from comp29msg.msg          import CommunicationInfo, DetectionResult, DetectionArray, UAVInfo

class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []
        self.result = 0

    def add_sample(self, new_sample):
        self.data.append(new_sample)
        # 保持数据窗口不超过窗口大小
        if len(self.data) > self.window_size:
            self.data.pop(0)
        self.result = self.__get_average()
        

    def get_value(self, new_sample = None):
        if new_sample is not None:
            self.add_sample(new_sample)
        return self.result
    
    def __get_average(self):
        return sum(self.data) / len(self.data) if len(self.data) != 0 else 0

TARGET_POS_LOCAL = np.array([
    [1.8, 1.8],
    [0, 0],
    [3.6, 0],
    [1.8, -1.8],
], dtype=np.float64)

"""
    1
  / | \
2------ 3
  \ | /
    4

"""
ADJ_MTX = np.array([
#    1  2  3  4
    [0, 1, 1, 1],  # 1
    [1, 0, 1, 0],  # 2
    [1, 1, 0, 0],  # 3
    [1, 1, 1, 0],  # 4
], dtype=np.int64)

# ADJ_MTX = np.array([
# #    1  2  3
#     [0, 1, 1],  # 1
#     [1, 0, 1],  # 2
#     [1, 1, 0],  # 3

# ], dtype=np.int64)


class FormationController:
    def __init__(self, target_pos, adj_mtx, uav_id_from_1, k=1) -> None:
        self.index_id = uav_id_from_1-1
        self.target_pos = target_pos
        self.adj_mtx = adj_mtx
        self.target_pos_local = self.target_pos-self.target_pos[self.index_id]
        self.target_dist = np.linalg.norm(self.target_pos_local, axis=1)
        print(self.target_pos_local)
        print(self.target_dist)
        self.k = k
        pass

    def update(self, dist):
        """_summary_
        ctrl = k * target_pos_local * adj_mtx * (dist - target_dist)
        Args:
            dist (list): distance between each drone
        """
        # print(self.target_pos_local)
        # print(self.adj_mtx[self.index_id, :])
        # print(dist)
        # print(self.target_dist)
        vel_local = self.k * np.dot(np.transpose(self.target_pos_local),
                                    np.transpose(np.multiply( self.adj_mtx[self.index_id, :],
                                            (dist - self.target_dist))))
        # print(dist - self.target_dist)
        vel_local_frd = [vel_local[1], vel_local[0]]
        return np.array(vel_local_frd)
        print(vel_local)
        # print(np.multiply( self.adj_mtx[self.index_id, :],
        #                                     (dist - self.target_dist)))
        pass
    
class Comp29MainNode(Node):
    def __init__(self, args):
        super().__init__('main_planner_node')
        
        # 参数
        self.uav_id = int(os.environ.get('e', '-1'))
        if self.uav_id == -1:
            self.get_logger().warn("UAV_ID未被设置！！！！")
        self.ctrler = FormationController(TARGET_POS_LOCAL, ADJ_MTX, self.uav_id, k=0.1)
        
        # 变量
        self.mission_state = MissionState.MIS_WAIT
        
        self.local_pos_est = None
        self.local_pos_est_got = False
        
        self.dist = None
        self.dist_got = False
        
        self.group = None
        self.group_got = False
        
        self.vel = [0., 0., 0.]
        
        self.lidar_height = 0.0
        
        self.arm_state = False
        
        self.detected_results = DetectedResultList()
        self.number_detected_state = False
        self.detected_got_time = None
        
        self.local_pos_int = [0,0]
        self.local_pos_dists = []
        self.dists = None
        self.dist_got = False
        self.pos_got = False
        
        self.opt_pos_x_filter = MovingAverageFilter()
        self.opt_pos_y_filter = MovingAverageFilter()
        
        self.vel_set_x_filter = MovingAverageFilter(4)
        self.vel_set_y_filter = MovingAverageFilter(4)
        
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
        )
        
        # 发布消息
        self.vel_frd_pub            = self.create_publisher(TwistStamped,   '/vel_set_frd', qos_profile)
        self.vel_ned_pub            = self.create_publisher(TwistStamped,   '/vel_set_ned', qos_profile)
        self.arm_cmd_pub            = self.create_publisher(Bool,           '/arm',         qos_profile)
        
        
        
        # 创建订阅者
        self.local_position_ned_est_sub        = self.create_subscription(
            PoseStamped, 
            '/local_position_ned_est',  
            self.local_pos_est_cb, 
            qos_profile
        )
        
        self.detection_result_sub        = self.create_subscription(
            DetectionArray, 
            '/number_detected',  
            self.number_detected_cb, 
            qos_profile
        )
        
        self.local_pos_sub        = self.create_subscription(
            PoseStamped, 
            '/local_position_ned', 
            self.local_pos_cb, 
            qos_profile
        )
        
        self.dist_sub        = self.create_subscription(
            Float32MultiArray, 
            '/uwb_filtered', 
            self.dist_cb, 
            qos_profile
        )
        
        self.vel_ned_sub        = self.create_subscription(
            TwistStamped, 
            '/velocity_and_angular', 
            self.vel_cb, 
            qos_profile
        )
        # print("ready")
        self.get_logger().info("ready")
        
        self.SIGNAL_LOST_TIME = 0.4
        self.watch_timer = self.create_timer(0.2, self.watch)
    def watch(self):
        if self.detected_got_time is not None:
            if time.time() - self.detected_got_time > self.SIGNAL_LOST_TIME:
                if self.number_detected_state:
                    self.get_logger().info(f"DETECT SIGNAL {color_codes['red']}LOST{color_codes['reset']}")
                self.number_detected_state = False
            else:
                if not self.number_detected_state:
                    self.get_logger().info(f"DETECT SIGNAL {color_codes['green']}BACK{color_codes['reset']}")
                self.number_detected_state = True
                
    def local_pos_est_cb(self, msg):
        pass
    def number_detected_cb(self, msg: DetectionResult):
        # self.number_detected_state = True
        # print(111)
        self.detected_results.msg2result(msg)
        self.detected_got_time = time.time()
        pass
    
    def vel_cb(self, msg: TwistStamped):
        self.vel = [msg.twist.linear.x, msg.twist.linear.y]
        self.vel_got = True
        self.vel_got_time = time.time()
        
    def dist_cb(self, msg:Float32MultiArray):
        # print(msg)
        self.dists = msg.data[self.uav_id*msg.layout.dim[0].size+1:self.uav_id*msg.layout.dim[0].size+5]
        self.dist_got = True
        # print(11)
        pass
    
    def local_pos_cb(self, msg: PoseStamped):
        self.local_pos = [msg.pose.position.x, msg.pose.position.y]
        self.pos_got = True
        # print(22)
        pass
    
    def formation_ctrl(self):
        """
        编队控制
        """
        pass
    
    def land_ctrl(self):
        """
        降落控制
        """
        pass
    
    def rend_ctrl(self):
        """
        聚合控制
        """
        pass
    
    def run(self):
        # 等待初始化与网络连接
        
        # 等待消息
        
        # 解锁
        
        # 起飞
        
        # 执行任务
        while True:
            rclpy.spin_once(self)
            if self.dist_got:
                break
            else:
                print("waiting for dist msgs")
                time.sleep(0.2)
        r_hat = np.array([5, 0])
        vv = r_hat / np.linalg.norm(r_hat)
        MAX_SIZE = 3000
        TIME_STEP = 0.1
        MAX_SPD = 0.3
        MIN_SPD = 0.08
        MIN_DIST = 1.1
        while rclpy.ok():
            rclpy.spin_once(self)
            # 根据状态执行任务
            if self.mission_state == MissionState.MIS_WAIT:
                pass
            elif self.mission_state == MissionState.MIS_WAIT:
                pass
            vv = np.array([0.0, 0.0])
            vv = vv if np.linalg.norm(vv) < MAX_SPD else vv*MAX_SPD/np.linalg.norm(vv)
            vv[0] = self.vel_set_x_filter.get_value(vv[0])
            vv[1] = self.vel_set_y_filter.get_value(vv[1])
            vel_ned_msg = TwistStamped()
            vel_ned_msg.twist.linear.x = vv[0] if math.fabs(vv[0]) > MIN_SPD else 0.0
            vel_ned_msg.twist.linear.y = vv[1] if math.fabs(vv[1]) > MIN_SPD else 0.0
            self.get_logger().info(
                f"d:{self.dists[0] :6.2f} {self.dists[1] :6.2f} {self.dists[2] :6.2f} {self.dists[3] :6.2f} == " + 
                f"v frd: {vv[0]:4.2f}, {vv[1]:4.2f}"
            )
            self.vel_ned_pub.publish(vel_ned_msg)
            # rclpy.spin_once(self)
            time.sleep(0.1)
        

def main(args=None):
    rclpy.init()
    
    formation_node = Comp29MainNode(args)
    
    try:
        formation_node.run()
    finally:
        print("formation 线程结束")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
