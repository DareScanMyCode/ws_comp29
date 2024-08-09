# 核心规划器
# 核心规划器
# from .gport_sdk.goprt import GportGimbal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import time
from quaternions import Quaternion as Quaternion
import math
import json
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
import numpy as np
from scipy.optimize import minimize
from .utils import *
from .formation_agent       import *
from sensor_msgs.msg        import Imu
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import Float32MultiArray
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped, Vector3
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
    [1.8, 5.0],
    [1.8, 2.5],
    [ 0.,  0.],
    [3.6, 0.],
], dtype=np.float64)

"""
       1
     / | \
    /  |  \
   /   2   \
  /  /   \  \
 //         \\
3-------------4

"""
ADJ_MTX = np.array([
#    1  2  3  4
    [0, 0, 0, 0],  # 1
    [1, 0, 0, 0],  # 2
    [1, 1, 0, 0],  # 3
    [1, 1, 1, 0],  # 4
], dtype=np.int64)


class Comp29MainNode(Node):
    def __init__(self, args):
        super().__init__('main_planner_node')
        self.logger = self.get_logger()
        
        # 参数
        self.uav_id = int(os.environ.get('UAV_ID', '-1'))
        self.logger.info(f"{color_codes['green']} UAV ID {self.uav_id} {color_codes['reset']}")
        if self.uav_id == -1:
            self.get_logger().warn("UAV_ID未被设置！！！！")
        self.mis_cfg_file = '/home/cat/ws_comp29/src/configs/missionCFG.json'
        # self.mis_cfg_file = '/home/zbw/ws_comp29/src/configs/missionCFG.json'
        self.mis_cfg = self.load_config(self.mis_cfg_file)
        self.log_config(self.mis_cfg)
        
        if self.mis_cfg is not None:
            self.map_w              = self.mis_cfg['MAP_W']
            self.map_l              = self.mis_cfg['MAP_L']
            self.map_angle          = self.mis_cfg['MAP_ANGLE']
            self.leader_id          = self.mis_cfg['LEADER_ID']
            self.angle_leader_id    = self.mis_cfg['ANGLE_LEADER_ID']
            self.num_uav            = self.mis_cfg['NUM_UAV']
            
            self.group_1            = self.mis_cfg['GROUP_1']
            self.group_2            = self.mis_cfg['GROUP_2']
            self.group_3            = self.mis_cfg['GROUP_3']
            
            self.group = 1 if self.uav_id in self.group_1 else -1
            self.group = 2 if self.uav_id in self.group_2 else -1 
            self.group = 3 if self.uav_id in self.group_3 else -1
            
            if self.group == -1:
                self.get_logger().warn("UAV_ID未被分组！！！！")
        else:
            self.get_logger().warn("未能读取到任务配置文件！！！！")
            
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
        
        # 编队控制器初始化
        if self.uav_id == 1:
            self.form_ctrler = Leader(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL)
        elif self.uav_id == 2:
            self.form_ctrler = AngleLeader(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL, 1-1)
        else:
            self.form_ctrler = Follower(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL, 0.1)
        
        # 消息控制
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
        
        # color部分
        self.color_dx = 0.
        self.color_dy = 0.
        self.last_color_got_time = None
        self.color_lost = True
        
        self.color_dx_sub       = self.create_subscription(
            Vector3, 
            '/color_offset', 
            self.color_dx_cb, 
            qos_profile
        )
        # print("ready")
        
        
        # 信号监视器
        self.SIGNAL_LOST_TIME = 0.4
        self.watch_timer = self.create_timer(0.2, self.watch)
        
        self.get_logger().info(f"{color_codes['green']}主控制器准备好了{color_codes['reset']}")
        
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
        if self.last_color_got_time is not None:
            if time.time() - self.last_color_got_time > self.SIGNAL_LOST_TIME:
                if not self.color_lost:
                    self.get_logger().info(f"COLOR SIGNAL {color_codes['red']}LOST{color_codes['reset']}")
                self.color_lost = True
            else:
                if self.color_lost:
                    self.get_logger().info(f"COLOR SIGNAL {color_codes['green']}BACK{color_codes['reset']}")
                self.color_lost = False
                
    def load_config(self, filepath):
        # 检查文件是否存在
        if not os.path.exists(filepath):
            self.logger.error(f"Config file {filepath} does not exist.")
            return None
        
        # 打开并读取 JSON 文件
        try:
            with open(filepath, 'r') as file:
                config = json.load(file)
            self.logger.info(f"Config file {filepath} loaded successfully.")
            return config
        except Exception as e:
            self.logger.error(f"Failed to read config file: {e}")
            return None
    
    def log_config(self, config):
        if config is None:
            self.logger.error("No configuration to log.")
            return
        
        # 使用 ROS2 logger 打印配置
        for key, value in config.items():
            if isinstance(value, dict):
                self.logger.info(f"{key}:")
                for sub_key, sub_value in value.items():
                    self.logger.info(f"  {sub_key}: {sub_value}")
            else:
                self.logger.info(f"{key}: {value}")
        self.logger.info(f">>> {color_codes['green']}MISSION CONFIG READ{color_codes['reset']}")
        
    def local_pos_est_cb(self, msg):
        pass
    
    def number_detected_cb(self, msg: DetectionResult):
        # self.number_detected_state = True
        # print(111)
        self.detected_results.msg2result(msg)
        self.detected_got_time = time.time()
        pass
    
    def color_dx_cb(self, msg:Vector3):
        self.color_dx = msg.x
        self.color_dy = msg.y
        self.last_color_got_time = time.time()
        
    def vel_cb(self, msg: TwistStamped):
        self.vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.vel_got = True
        self.vel_got_time = time.time()
        
    def dist_cb(self, msg:Float32MultiArray):
        # print(msg)
        self.dists = msg.data[self.uav_id*msg.layout.dim[0].size+1:self.uav_id*msg.layout.dim[0].size+self.num_uav+1]
        self.dist_got = True
        self.logger.info(f"{self.uav_id}: {self.uav_id*msg.layout.dim[0].size+1}, {self.uav_id*msg.layout.dim[0].size+self.num_uav+1}")
        # print(11)
        pass
    
    def local_pos_cb(self, msg: PoseStamped):
        self.local_pos = [msg.pose.position.x, msg.pose.position.y]
        self.pos_got = True
        # print(22)
        pass
    
    def leader_ctrl(self):
        """
        Leader 控制，负责绕圈圈
        """
        pass
    def formation_ctrl(self):
        """
        编队控制
        """
        if self.uav_id == 2:
            if not self.color_lost:
                vv_frd = self.form_ctrler.get_ctrl(self.dists, self.color_dx)
            else:
                vv_frd = self.form_ctrler.get_ctrl(self.dists)
        elif self.uav_id == self.leader_id:
            vv_frd = self.form_ctrler.get_ctrl(self.dists)
            vv_frd = self.leader_ctrl()
        else:
            # Follower
            vv_frd = self.form_ctrler.get_ctrl(self.dists)
        self.logger.debug(f"[FORM] vv_frd: {vv_frd[0]}, {vv_frd[1]}")
        return vv_frd
    
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
        MIN_SPD = 0.02
        MIN_DIST = 1.1
        self.mission_state = MissionState.MIS_FORMATION_FOLLOW
        while rclpy.ok():
            rclpy.spin_once(self)
            # 根据状态执行任务
            if self.mission_state == MissionState.MIS_WAIT:
                pass
            elif self.mission_state == MissionState.MIS_FORMATION_LEADER:
                pass
            elif self.mission_state == MissionState.MIS_FORMATION_FOLLOW:
                
                vv_frd = self.formation_ctrl()
                pass
            elif self.mission_state == MissionState.MIS_FORMATION_LEAVING:
                pass
            elif self.mission_state == MissionState.MIS_BASE:
                pass
            elif self.mission_state == MissionState.MIS_BASE_STAND:
                pass
            elif self.mission_state == MissionState.MIS_BASE_MOVE:
                pass
            elif self.mission_state == MissionState.MIS_TARGET_WATCHING:
                pass
            elif self.mission_state == MissionState.MIS_LANDING:
                pass
            
            vv = vv_frd if isinstance(vv_frd, np.ndarray) else np.array(vv_frd)
            vv = vv if np.linalg.norm(vv) < MAX_SPD else vv*MAX_SPD/np.linalg.norm(vv)
            vv[0] = self.vel_set_x_filter.get_value(vv[0])
            vv[1] = self.vel_set_y_filter.get_value(vv[1])
            vel_frd_msg = TwistStamped()
            vel_frd_msg.twist.linear.x = vv[0] if math.fabs(vv[0]) > MIN_SPD else 0.0
            vel_frd_msg.twist.linear.y = vv[1] if math.fabs(vv[1]) > MIN_SPD else 0.0
            self.get_logger().info(
                f"d:{self.dists[0] :6.2f} {self.dists[1] :6.2f} {self.dists[2] :6.2f} {self.dists[3] :6.2f} == " + 
                f"v frd: {vv[0]:4.2f}, {vv[1]:4.2f}"
            )
            self.vel_frd_pub.publish(vel_frd_msg)
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
