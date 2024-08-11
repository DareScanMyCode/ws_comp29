# 核心规划器
# 核心规划器
# from .gport_sdk.goprt import GportGimbal
from comp29planner.land_controller import LandController
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
from comp29planner.utils import *
from comp29planner.formation_agent       import *
from comp29planner.height_ctrl           import HightCtrl
from sensor_msgs.msg        import Imu
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import Float32MultiArray
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped, Vector3
from std_msgs.msg           import Int64, Float64, Bool, Float32
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
    [1.6, 4.2],
    [1.6, 2.0],
    [ 0.,  0.],
    [3.2, 0.],
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
        
        # 读取任务配置文件
        self.username = os.environ.get("USER", 'cat')
        self.mis_cfg_file = f'/home/{self.username}/ws_comp29/src/configs/missionCFG.json'
        # self.mis_cfg_file = '/home/zbw/ws_comp29/src/configs/missionCFG.json'
        self.mis_cfg = self.load_mis_cfg(self.mis_cfg_file)
        self.log_mis_cfg(self.mis_cfg)
        
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
            
            self.group = 1 if self.uav_id in self.group_1 else 2 if self.uav_id in self.group_2 else 3 if self.uav_id in self.group_3 else -1
            
            if self.group == -1:
                self.get_logger().warn("UAV_ID未被分组！！！！")
                
            self.uav_speed          = self.mis_cfg['UAV_SPEED']
            self.filght_height      = self.mis_cfg['FLIGHT_HEIGHT']
            self.min_dist           = self.mis_cfg['MIN_DIST']
            self.uav_max_speed      = self.mis_cfg['UAV_MAX_SPEED']
        else:
            self.get_logger().warn("未能读取到任务配置文件！！！！")
        
        if self.uav_id == self.leader_id:
            self.logger.info(f">>> {color_codes['green']} UAV {self.uav_id} 被配置为 LEADER {color_codes['reset']}")
            self.formation_role = AgentRole.ROLE_FORMATION_LEADER
        elif self.uav_id == self.angle_leader_id:
            self.logger.info(f">>> {color_codes['green']} UAV {self.uav_id} 被配置为 ANGLE LEADER {color_codes['reset']}")
            self.formation_role = AgentRole.ROLE_FORMATION_ANGLE_LEADER
        else:
            self.logger.info(f">>> {color_codes['green']} UAV {self.uav_id} 被配置为 FOLLOWER {color_codes['reset']}")
            self.formation_role = AgentRole.ROLE_FORMATION_FOLLOWER
            
        # 变量
        self.mission_state = MissionState.MIS_WAIT
        
        self.local_pos_est = None
        self.local_pos_est_got = False
        
        self.dist = None
        self.dist_got = False
        
        self.group = None
        self.group_got = False
        
        self.vel = [0., 0., 0.]
        
        self.lidar_height = None
        
        self.arm_state = False
        
        self.detected_result = DetectedResult()
        
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
        
        # 其他无人机的信息
        self.uav_infos = [UAVInfo() for _ in range(self.num_uav)]
        
        
        # 高度控制器初始化
        self.height_ctrler = HightCtrl(tgt_height=self.filght_height)
        # 编队控制器初始化
        if self.uav_id == 1:
            self.form_ctrler = Leader(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL)
        elif self.uav_id == 2:
            self.form_ctrler = AngleLeader(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL, 0)
        else:
            self.form_ctrler = Follower(self.uav_id-1, ADJ_MTX, TARGET_POS_LOCAL, 0.1)
        
        # 聚集控制器初始化
        self.landmark_uav_id = None
        
        # 降落控制器初始化
        self.land_ctrler = LandController()
        self.final_land_flag = False
        
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
        comm_topic_name = '/uav' + str(self.uav_id) + '/comm_info'
        self.logger.info(f"[MAIN] comm_topic_name: {comm_topic_name}")
        self.comm_info_sub        = self.create_subscription(
            CommunicationInfo, 
            comm_topic_name,  
            self.comm_info_cb, 
            qos_profile
        )
        
        self.local_position_ned_est_sub        = self.create_subscription(
            PoseStamped, 
            '/local_position_ned_est',  
            self.local_pos_est_cb, 
            qos_profile
        )
        
        self.detection_result_sub        = self.create_subscription(
            # DetectionArray, 
            DetectionResult, 
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
        
        self.lidar_height_sub   = self.create_subscription(
            Float32, 
            '/lidar_height', 
            self.lidar_cb, 
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
        """
        判断数据是否过时
        """
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
                
    def load_mis_cfg(self, filepath):
        """
        加载任务配置文件
        """
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
    
    def log_mis_cfg(self, config):
        """
        打印任务配置文件
        """
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
    
    def lidar_cb(self, msg:Float32):
        # return
        self.lidar_height = float(msg.data)
        
    def comm_info_cb(self, msg: CommunicationInfo):
        # TODO 测试！
        for uav_info in msg.uav:
            self.uav_infos[uav_info.id] = uav_info
        self.tracking_list = msg.tracking_list.data
        self.dist_mat = msg.dist_mat.data
        self.guard_list = msg.guard_list.data
        # TODO 更新本机变量
        
    
    def number_detected_cb(self, msg: DetectionResult):
        # self.number_detected_state = True
        # print(111)
        self.detected_result.msg2result(msg)
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
        self.dists = msg.data[self.uav_id*msg.layout.dim[0].size+1:self.uav_id*msg.layout.dim[0].size+self.num_uav+1]
        self.dist_got = True
        self.logger.info(f"{self.uav_id}: {self.uav_id*msg.layout.dim[0].size+1}, {self.uav_id*msg.layout.dim[0].size+self.num_uav+1}")
    
    def local_pos_cb(self, msg: PoseStamped):
        self.local_pos = [msg.pose.position.x, msg.pose.position.y]
        self.pos_got = True
    
    def leader_ctrl(self):
        """
        Leader 控制，负责绕圈圈
        往前 直到 y = self.map_l
        往右 10m
        向后 直到 y = 5m
        """
        pass
    
    def formation_ctrl(self):
        """
        编队控制
        """
        if self.formation_role == AgentRole.ROLE_FORMATION_ANGLE_LEADER:
            if not self.color_lost:
                vv_frd = self.form_ctrler.get_ctrl(self.dists, self.color_dx)
            else:
                vv_frd = self.form_ctrler.get_ctrl(self.dists)
        elif self.formation_role == AgentRole.ROLE_FORMATION_LEADER:
            # 调用leader control??还是用类？
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
        # 应该先飞到目标位置与高度，然后直接降落，需要一个flag
        vv_frd = [0., 0., 0.]
        if self.final_land_flag:
            self.logger.debug(f"[LAND] final land flag is True")
            vv_frd = [0., 0., 0.15]
            return vv_frd
        else:
            if self.number_detected_state:
                if self.detected_result.detected_num == self.group:
                    # TODO 确认pose
                    vv_frd = self.land_ctrler.land_ctrl(self.detected_result.detected_pose[0], self.detected_result.detected_pose[1])
                    self.logger.debug(f"[LAND] detected num: {self.detected_result.detected_num}, group: {self.group}, vel: {vv_frd}")
                    return vv_frd
                else:
                    self.logger.debug(f"[LAND] 检测到目标{self.detected_result.detected_num}，但不是目标{self.group}")
                    return vv_frd
            else:
                self.logger.debug(f"未检测到目标")
                # TODO 附近瞎逛或者离检测到的飞机近一点
                return vv_frd
        pass
    
    def rend_ctrl(self):
        """
        聚合控制
        """
        # 距离远的时候使用之前的估计位置
        # 距离近的时候使用rendezvous方法
        if self.dists[self.landmark_uav_id] > 5.:
            # TODO 应该是距离估计比较准之后切换
            # 使用之前的估计位置
            pass
        else:
            # 使用rendezvous方法
            pass
        pass
    
    def uav_setup(self):
        # 等待初始化与网络连接
        
        # 等待消息
        
        # 解锁
        self.arm_cmd_pub.publish(Bool(data=True))
        # 起飞
        pass
        
    def run(self):
        # 初始化无人机
        # self.uav_setup()
        
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
            
            vz = self.height_ctrler.get_ctrl(self.lidar_height)
            
            vel_frd_msg = TwistStamped()
            vel_frd_msg.twist.linear.x = vv[0] if math.fabs(vv[0]) > MIN_SPD else 0.0
            vel_frd_msg.twist.linear.y = vv[1] if math.fabs(vv[1]) > MIN_SPD else 0.0
            vel_frd_msg.twist.linear.z = vz
            
            
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
