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

from sensor_msgs.msg import Imu

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
    [1.6,    1.8],
    [1.6,    0.0],
    [ 0.,    0.3],
    [3.2,    0.3],
], dtype=np.float64)

LEAVING_ID = []

"""
X      1
     / | \
   /   |   \
 /     |     \
3------2-------4 Y
"""

"""
       1
     / | \
   /   |  \
 /     |   \
3------2    \
         -\  \
            -\\
               4 
"""
ADJ_MTX = np.array([
#    1  2  3  4
    [0, 0, 0, 0],  # 1
    [1, 0, 0, 0],  # 2
    [1, 1, 0, 0],  # 3
    [1, 1, 1, 0],  # 4
], dtype=np.int64)


## 8架
"""
1 2 3 4 5 6 7 8 9 0 1 2 3 4
          /-1-\            16
        /   |   \          14
      /     |     \        12
    3-------2------4       10
   / \    /  \    / \      8
  /   \ /     \  /   \     6
 /     5-\---/-6      \    4
/ ____/    X   \______ \   2
7--------/---\----------8  0

"""
TARGET_POS_LOCAL = np.array([
    [7,    16],
    [7,    10.0],
    [3.,   10.0],
    [11,   10.0],
    [5,    4.0],
    [9,    4.0],
    [1,    0.0],
    [13,   0.0],
], dtype=np.float64)

ADJ_MTX = np.array([
#    1  2  3  4  5  6  7  8
    [0, 0, 0, 0, 0, 0, 0, 0],  # 1
    [1, 0, 0, 0, 0, 0, 0, 0],  # 2
    [1, 1, 0, 0, 0, 0, 0, 0],  # 3
    [1, 1, 0, 0, 0, 0, 0, 0],  # 4
    [0, 1, 1, 1, 0, 0, 0, 0],  # 5
    [0, 1, 1, 1, 0, 0, 0, 0],  # 6
    [0, 1, 0, 0, 1, 1, 0, 0],  # 7
    [0, 0, 0, 1, 1, 1, 0, 0],  # 8
], dtype=np.int64)


## 6架
"""
1 2 3 4 5 6 7 8 9 0 1 2 3 4
          /-1-\            16
        /   |   \          14
      /     |     \        12
    3-------2-------4       10
   / \    /   \    / \      8
  /   \ /      \  /   \     6
 /  O  5-\-----/-6     \    4
"""
TARGET_POS_LOCAL = np.array([
    [2,    4],
    [2,    2.0],
    [0.,   2.0],
    [4,    2.0],
    [1,    0.0],
    [3,    0.0],
], dtype=np.float64)
TARGET_POS_LOCAL = TARGET_POS_LOCAL/2
TARGET_POS_LOCAL = np.array([
    [7,    16],
    [7,    10.0],
    [3.,   10.0],
    [11,   10.0],
    [5,    4.0],
    [9,    4.0],
], dtype=np.float64)
TARGET_POS_LOCAL = np.array([
    [2,    4],
    [2,    2.0],
    [0.,   2.0],
    [4,    2.0],
    [1,    0.0],
    [3,    0.0],
], dtype=np.float64)
ADJ_MTX = np.array([
#    1  2  3  4  5  6 
    [0, 0, 0, 0, 0, 0],  # 1
    [1, 0, 1, 1, 0, 0],  # 2 观测34是为了保险，怕颜色丢掉
    [1, 1, 0, 0, 0, 0],  # 3
    [1, 1, 0, 0, 0, 0],  # 4
    [0, 1, 1, 1, 0, 0],  # 5
    [0, 1, 1, 1, 0, 0],  # 6
], dtype=np.int64)


class Comp29MainNode(Node):
    def __init__(self, args):
        super().__init__('main_planner_node')
        self.logger = self.get_logger()
        self.declare_parameter('use_ekf_pos', True)
        self.declare_parameter('uav_id', -1)
        self.use_ekf_pos = self.get_parameter('use_ekf_pos').get_parameter_value().bool_value
        self.uav_id_ros  = self.get_parameter('uav_id').get_parameter_value().integer_value
        
        # 参数
        self.uav_id = int(os.environ.get('UAV_ID', '-1'))
        # TODO delete
        # self.uav_id = self.uav_id_ros
        if self.uav_id == -1:
            self.get_logger().warn("UAV_ID未被设置！！！！")
            self.uav_id = self.uav_id_ros
        
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
            self.dx_per_line        = self.mis_cfg['DX_PER_LINE']
            self.ori_dx_per_line    = self.dx_per_line
            self.leader_id          = self.mis_cfg['LEADER_ID']
            self.angle_leader_id    = self.mis_cfg['ANGLE_LEADER_ID']
            self.num_uav            = self.mis_cfg['NUM_UAV']
            
            self.group_1            = self.mis_cfg['GROUP_1']
            self.group_2            = self.mis_cfg['GROUP_2']
            self.group_3            = self.mis_cfg['GROUP_3']
            
            guards = self.mis_cfg['GUARDS']
            
            self.group = 1 if self.uav_id in self.group_1 else 2 if self.uav_id in self.group_2 else 3 if self.uav_id in self.group_3 else -1
            self.member_list = self.group_1 if self.group == 1 else self.group_2 if self.group == 2 else self.group_3 if self.group == 3 else []
            self.member_list = [i for i in self.member_list if i != self.uav_id]
            self.guard_id = guards[self.group-1]  # 本组的守卫id
            self.guarded_flag = 0
            self.ever_seen = [0., 0., 0., 0., 0.]

            if self.group == -1:
                self.get_logger().warn("UAV_ID未被分组！！！！")
            self.get_logger().warn(f"{ColorCodes.green} UAV{self.uav_id} 被分组为 {self.group}{ColorCodes.reset}")
                
            self.uav_speed          = self.mis_cfg['UAV_SPEED']
            self.filght_height      = self.mis_cfg['FLIGHT_HEIGHT']
            self.min_dist           = self.mis_cfg['MIN_DIST']
            self.uav_max_speed      = self.mis_cfg['UAV_MAX_SPEED']  # mot used
            
            # 找到初始位置
            self.local_pos_inital   = self.mis_cfg['INITIAL_POS'][f'UAV{self.uav_id}']   # 初始位置
            self.logger.info(f"INITIAL_POS: {self.local_pos_inital}")
            
        else:
            self.get_logger().warn("未能读取到任务配置文件！！！！")
        
        # 输出一些配置
        self.logger.info(f"{color_codes['green']} UAV ID {self.uav_id} {color_codes['reset']}")
        
        if self.use_ekf_pos:
            self.logger.info(f"{ColorCodes.green}DO USE EKF position.{ColorCodes.reset}")
        else:
            self.logger.info(f"{ColorCodes.red}DO NOT USE EKF position.{ColorCodes.reset}")
            
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
        self.mission_began = False
        self.mission_ready = False
        self.mission_state = MissionState.MIS_WAIT
        
        self.local_pos_est = [0., 0.]
        self.local_pos_est_got = False
        self.arrive_guard_pos = False
        self.dist = None
        self.dist_got = False
        
        self.group_got = True
        
        self.vel = [0., 0., 0.]
        self.tt11 = 0
        
        self.lidar_height = None
        self.imu_data=Node
        self.rpy=[0.0,0.0,0.0]
        
        self.arm_state = False
        
        self.detected_result = DetectedResult()
        
        self.number_detected_state = False
        self.detected_got_time = None
        self.local_pos_fcu = None
        self.local_pos_est = None
        self.local_pos_fcu_int_base = self.local_pos_inital         # 飞控位置累计基准，防止飞控位置归零
        self.local_pos_fcu      = self.local_pos_fcu_int_base   # 飞控位置累计位置
        self.local_pos_dists    = []
        self.dists = None
        self.dist_got = False
        self.pos_got = False
        self.all_target_found = False  # 所有目标是否都被找到
        self.local_pos_start = None
        
        self.opt_pos_x_filter = MovingAverageFilter()
        self.opt_pos_y_filter = MovingAverageFilter()
        
        self.vel_frd_set_x_filter = MovingAverageFilter(4)
        self.vel_frd_set_y_filter = MovingAverageFilter(4)
        
        self.vel_ned_set_x_filter = MovingAverageFilter(4)
        self.vel_ned_set_y_filter = MovingAverageFilter(4)
        self.time_to_leave = None
        # leader 信息
        self.leader_vel_ned = [0., 0., 0.]
        self.leader_target_index = 0
        self.leader_target_poss = [[-6.8, 1.5], [-6.8, 4.05], [-1.0, 4.05]]
        self.leader_target = [-7.35, 1.5]#ned
        self.tracking_list = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
        self.comm_ttt = 0
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
        
        # leader巡航配置
        self.direc = self.DIR_FRONT
        # 消息控制
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
        )
        
        # 发布消息
        self.pos_ned_fcu_int_pub            = self.create_publisher(PoseStamped,    '/pos_ned_fcu_int', qos_profile)
        self.vel_frd_pub                    = self.create_publisher(TwistStamped,   '/vel_set_frd',     qos_profile)
        self.vel_ned_pub                    = self.create_publisher(TwistStamped,   '/vel_set_ned',     qos_profile)
        self.arm_cmd_pub                    = self.create_publisher(Bool,           '/arm',             qos_profile)
        self.gimbal_pub                     = self.create_publisher(PoseStamped,    'gimbalrpy_setpoint', qos_profile)
        self.local_mis_sta_pub              = self.create_publisher(Int64,          '/mission_state',   qos_profile)
        # while rclpy.ok():
        #     self.local_mis_sta_pub.publish(Int64(data=self.uav_id))
        #     # self.logger.info(f"publish {self.uav_id}")
        #     time.sleep(0.5)
        # 创建订阅者
        ekf2_pos_topic_name = '/uav' + str(self.uav_id) + "/ekf2/pose"
        comm_topic_name = '/uav' + str(self.uav_id) + '/comm_info'
        self.logger.debug(f"[MAIN] comm_topic_name: {comm_topic_name}")
        self.comm_info_sub        = self.create_subscription(
            CommunicationInfo, 
            comm_topic_name,  
            self.comm_info_cb, 
            qos_profile
        )
        
        self.local_position_ned_fcu_sub   = self.create_subscription(
            PoseStamped,
            '/local_position_ned',
            self.fcu_pos_cb,
            qos_profile
        )
        
        self.local_position_ned_est_sub        = self.create_subscription(
            PoseStamped, 
            ekf2_pos_topic_name,  
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
        
        # self.local_pos_sub        = self.create_subscription(
        #     PoseStamped, 
        #     '/local_position_ned', 
        #     self.local_pos_cb, 
        #     qos_profile
        # )
        
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
            Float64, 
            '/lidar_height', 
            self.lidar_cb, 
            qos_profile
        )
        
        self.imu_sub=self.create_subscription(
            Imu,
            '/imu',
            self.imu_cb,
            qos_profile
        )
        
        self.mis_sta_sub = self.create_subscription(
            Int64,
            '/super_mission_state',
            self.sup_mis_sta_cb,
            qos_profile
        )
        
        self.guard_pos_ned_est = None
        self.guard_pos_ned_est_sub = self.create_subscription(
            PoseStamped,
            '/guard_pos_est_ned',
            self.guard_pos_ned_est_cb,
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
        
        
        # 信号监视器
        self.SIGNAL_LOST_TIME = 0.9
        self.watch_timer = self.create_timer(0.2, self.watch)
        self.timer_1s    = self.create_timer(1.0, self.timer_1s_cb)
        self.get_logger().info(f"{color_codes['green']}主程序准备好了{color_codes['reset']}")
    
    def timer_1s_cb(self):
        """
        1s运行一次
        """
        self.gimbal_ctrl()
        
    def gimbal_ctrl(self):
        """
        1s运行一次
        对于angle leader，云台向前
        对于其他，云台向下，pitch为90度
        """
        gim_msg = PoseStamped()
        if self.formation_role == AgentRole.ROLE_FORMATION_ANGLE_LEADER:
            gim_msg.header.stamp = self.get_clock().now().to_msg()
            if self.mission_state == MissionState.MIS_FORMATION or self.mission_state in [MissionState.MIS_WAIT, MissionState.MIS_READY, MissionState.MIS_BEGIN]:
                gim_msg.pose.position.x = 0.0
                gim_msg.pose.position.y = 0.0
                gim_msg.pose.position.z = 0.0
            else:
                gim_msg.pose.position.x = 0.0
                gim_msg.pose.position.y = -90.0
                gim_msg.pose.position.z = 0.0
        else:
            gim_msg.header.stamp = self.get_clock().now().to_msg()
            gim_msg.pose.position.x = 0.0
            gim_msg.pose.position.y = -90.0
            gim_msg.pose.position.z = 0.0
        self.gimbal_pub.publish(gim_msg)
        # self.logger.info(f"[MAIN] =============gimbal ctrl: {gim_msg.pose.position.x}, {gim_msg.pose.position.y}, {gim_msg.pose.position.z}")
        
    def watch(self):
        """
        判断数据是否过时
        """
        self.local_mis_sta_pub.publish(Int64(data=self.mission_state))
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
        self.logger.info(f">>> {color_codes['green']}============ MISSION CONFIG BEGIN ============{color_codes['reset']}")
        # 使用 ROS2 logger 打印配置
        for key, value in config.items():
            if isinstance(value, dict):
                self.logger.info(f"{key}:")
                for sub_key, sub_value in value.items():
                    self.logger.info(f"  {sub_key}: {sub_value}")
            else:
                self.logger.info(f"{key}: {value}")
        self.logger.info(f">>> {color_codes['green']}============= MISSION CONFIG END ============={color_codes['reset']}")
        # self.logger.info(f">>> {color_codes['green']}MISSION CONFIG READ{color_codes['reset']}")
    
    def guard_pos_ned_est_cb(self, msg: PoseStamped):
        self.guard_pos_ned_est = [msg.pose.position.x, msg.pose.position.y]
        self.tt11+=1
        if self.tt11 % 10 == 0:
            self.logger.info(f"[MAIN] guard pos est: {self.guard_pos_ned_est[0]:.2f}, {self.guard_pos_ned_est[1]:.2f}")
        
        
    K_GOTO = 0.5
    def goto_pos_NED(self, x, y, height=None):
        # 注意坐标系！
        vv_ned = [0.0, 0.0, 0.0]
        if self.local_pos_est is not None:
            # TODO 
            vv_ned[0] = self.K_GOTO * (x - self.local_pos_est[0])
            vv_ned[1] = self.K_GOTO * (y - self.local_pos_est[1])
            if height is None:
                height = self.filght_height
            self.height_ctrler.tgt_height = height
        else:
            self.logger.info(f"[MAIN] local pos est is None")
        return vv_ned
    
    GOTO_POS_ERR  = 0.4
    def goto_pose_ready(self, x, y, z=None):
        if abs(x - self.local_pos_est[0]) > self.GOTO_POS_ERR:
            return False
        if abs(y - self.local_pos_est[1]) > self.GOTO_POS_ERR:
            return False
        return True
        
    def sup_mis_sta_cb(self, msg:Int64):
        # self.logger.info(f"GOT MISSSSSSSSSS {msg.data}")
        if msg.data == MissionState.MIS_END:
            self.mission_state = MissionState.MIS_END
            self.logger.info(f"[MAIN] got MISSION {ColorCodes.yellow}END END END{ColorCodes.reset}")
        elif msg.data == MissionState.MIS_STOP:
            self.mission_state = MissionState.MIS_STOP
            self.logger.info(f"[MAIN] got MISSION {ColorCodes.red}STOP STOP STOP{ColorCodes.reset}")
        elif msg.data == MissionState.MIS_READY and not self.mission_ready:
            self.mission_ready = True
            self.mission_state = MissionState.MIS_READY
            self.logger.info(f"[MAIN] got MISSION {ColorCodes.red}READY READY READY{ColorCodes.reset}")
        elif msg.data == MissionState.MIS_BEGIN and not self.mission_began:
            self.mission_state = MissionState.MIS_BEGIN
            self.mission_began = True
            self.logger.info(f"[MAIN] {ColorCodes.green}任务开始{ColorCodes.reset}")

    POS_RESET_ERR = 0.2
    def fcu_pos_cb(self, msg:PoseStamped):
        if self.local_pos_fcu is None:
            self.local_pos_fcu = [msg.pose.position.x + self.local_pos_fcu_int_base[0],
                                  msg.pose.position.y + self.local_pos_fcu_int_base[1], 
                                  msg.pose.position.z]
        else:
            if abs(msg.pose.position.x) < self.POS_RESET_ERR and abs(msg.pose.position.y) < self.POS_RESET_ERR:
                # 如果飞控位置归零，更新累计位置的基准位置为前一时刻的估计位置
                # 还要判断两个时刻的差值
                if abs(msg.pose.position.x - self.local_pos_fcu[0]) > self.POS_RESET_ERR and abs(msg.pose.position.y - self.local_pos_fcu[1]) > self.POS_RESET_ERR:
                    # 更新基准位置
                    self.local_pos_fcu_int_base = [self.local_pos_fcu[0], self.local_pos_fcu[1]]
            self.local_pos_fcu = [msg.pose.position.x + self.local_pos_fcu_int_base[0],
                                  msg.pose.position.y + self.local_pos_fcu_int_base[1], 
                                  msg.pose.position.z]
        if not self.use_ekf_pos:
            self.local_pos_est = self.local_pos_fcu
        fcu_int_pose = PoseStamped()
        fcu_int_pose.pose.position.x = self.local_pos_fcu[0]
        fcu_int_pose.pose.position.y = self.local_pos_fcu[1]
        fcu_int_pose.pose.position.z = self.local_pos_fcu[2]
        self.pos_ned_fcu_int_pub.publish(fcu_int_pose)
        # # TODO 删掉
        # self.local_pos_est = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
    def local_pos_est_cb(self, msg: PoseStamped):
        if self.use_ekf_pos:
            self.local_pos_est = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            # self.logger.info(f"[MAIN] local pos est: {self.local_pos_est[0]:.2f}, {self.local_pos_est[1]:.2f}, {self.local_pos_est[2]:.2f}")
        pass
    
    def lidar_cb(self, msg:Float64):
        # return
        self.lidar_height = float(msg.data)
    
    def imu_cb(self,msg):
        self.imu_data=msg
        q = Quaternion(w=self.imu_data.orientation.w, x=self.imu_data.orientation.x, y=self.imu_data.orientation.y, z=self.imu_data.orientation.z)
        euler=q.get_euler()
        self.rpy=euler
        # self.get_logger().info(
        #         f"rpy:{self.rpy[0] :6.2f} {self.rpy[1] :6.2f} {self.rpy[2] :6.2f} "
        #     )     
    
    def comm_info_cb(self, msg: CommunicationInfo):
        # TODO 测试！
        # 其他飞机信息
        
        for uav_info in msg.uav:
            # self.get_logger().info(
            #     f'---------{uav_info.id}: {uav_info.mission_stat}------------'
            # )
            if uav_info.id == self.leader_id and self.formation_role != AgentRole.ROLE_FORMATION_LEADER:
                self.leader_vel_ned = [uav_info.vel.twist.linear.x, uav_info.vel.twist.linear.y, 0.0]
            self.uav_infos[uav_info.id-1] = uav_info
            if uav_info.mission_stat == -1 or uav_info.id == 0:
                # 沒有任务状态，空数据
                continue
            if uav_info.mission_stat == MissionState.MIS_FORMATION_LEAVING:
                if uav_info.id not in LEAVING_ID:
                    LEAVING_ID.append(uav_info.id)
                    ADJ_MTX[:][uav_info.id-1] = 0 # 改变脱离的邻接矩阵
                    self.form_ctrler.update_adj(ADJ_MTX)
                    self.get_logger().info(
                        f"id: {uav_info.id} is leaving"
                    )
                if len(LEAVING_ID) == 2:
                    self.dx_per_line=self.ori_dx_per_line/2
                
            # if self.mission_state == MissionState.MIS_TARGET_WATCHING and uav_info.mission_stat == MissionState.MIS_LANDING and uav_info.group_id in self.member:
            #     self.guarded_flag += 1

        if LEAVING_ID and 2 not in LEAVING_ID:
            if 3 in LEAVING_ID:
                TARGET_POS_LOCAL = np.array([
                    [1.6,    2.3],
                    [ 0.,    0.0],
                    [ 0.,    0. ], # 脱离
                    [3.2,    0.0],
                ], dtype=np.float64)
            else:
                TARGET_POS_LOCAL = np.array([
                    [1.6,    4.2],
                    [3.2,    1.2],
                    [ 0.,    1.2],
                    [ 0.,     0.], # 脱离
                ], dtype=np.float64)
            # self.form_ctrler.update_tgt_pos(TARGET_POS_LOCAL)
        # 本机信息
        
        # if self.group in [self.tracking_list[0], self.tracking_list[3], self.tracking_list[6]] and not self.group in self.ever_seen:
        #     self.tracking_flag = True
        #     self.tracking_list = msg.tracking_list.data
        tmp = [int(msg.tracking_list.data[0]), int(msg.tracking_list.data[3]), int(msg.tracking_list.data[6])]
        for i in range(3):
            if tmp[i] == -1:
                continue
            elif self.ever_seen[tmp[i]-1] == 0:
                tmp[i] -= 1
                self.ever_seen[tmp[i]] = 1
                self.tracking_list[tmp[i]*3] = msg.tracking_list.data[i*3]
                self.tracking_list[tmp[i]*3+1] = msg.tracking_list.data[i*3+1]
                self.tracking_list[tmp[i]*3+2] = msg.tracking_list.data[i*3+2]
        # for i in range(3):
        #     if i in [int(msg.tracking_list.data[0]), int(msg.tracking_list.data[3]), int(msg.tracking_list.data[6])] and self.ever_seen[i] == 0:
        #         self.ever_seen[i] = 1
        #         tracking_list = msg.tracking_list.data
        #         idx =  [index for index, element in enumerate([tracking_list[0],tracking_list[3],tracking_list[6]]) if int(element) == i]
        #         self.tracking_list[i*3] = msg.tracking_list.data[idx*3]
        #         self.tracking_list[i*3+1] = msg.tracking_list.data[idx*3+1]
        #         self.tracking_list[i*3+2] = msg.tracking_list.data[idx*3+2]
                
        if self.tracking_list[0] != -1 and self.tracking_list[3] != -1 and self.tracking_list[6] != -1 and not self.all_target_found:
            self.all_target_found = True # 只进一次,GOTO_GUARD之后要降落了
            if self.uav_id != self.guard_id:
                # 非守卫
                self.all_target_found = True
                self.mission_state = MissionState.MIS_GOTO_GUARD
                self.get_logger().info(
                    f" {ColorCodes.magenta} 所有目标被找到！ {ColorCodes.reset}"
                    )
        self.comm_ttt += 1
        if self.comm_ttt % 5 == 0:
            self.get_logger().info(f"target local: {self.tracking_list} ")
            self.get_logger().info(f"target msgg : {msg.tracking_list.data}")
        self.dist_mat = msg.dist_mat.data
        self.guard_list = msg.guard_list.data
        # TODO 更新本机变量
        if self.mission_state == MissionState.MIS_FORMATION and self.uav_id == self.leader_id and self.all_target_found == True:
            self.mission_state = MissionState.MIS_FORMATION_LEAVING
        if self.mission_state == MissionState.MIS_FORMATION and self.group in [self.tracking_list[0], self.tracking_list[3], self.tracking_list[6]] and self.uav_id != self.leader_id:
            self.mission_state = MissionState.MIS_FORMATION_LEAVING
            # self.get_logger().info("***********change my state************")
            # TARGET_POS_LOCAL[self.uav_id-1] = [0., 0.]
            ADJ_MTX[:][self.uav_id-1] = 0
            self.form_ctrler.update_adj(ADJ_MTX)
        
    def number_detected_cb(self, msg: DetectionResult):
        self.number_detected_state = True
        self.detected_result.detected_pose = [msg.position.x,msg.position.y]
        self.detected_result.detected_num = int(msg.object_name.data)

        self.detected_got_time = time.time()
    
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
        # self.logger.info(f"{self.uav_id}: {self.uav_id*msg.layout.dim[0].size+1}, {self.uav_id*msg.layout.dim[0].size+self.num_uav+1}")
    
    def local_pos_cb(self, msg: PoseStamped):
        self.local_pos = [msg.pose.position.x, msg.pose.position.y]
        self.pos_got = True
    
    DIR_FRONT   = 1
    DIR_BACK    = 2
    DIR_RIGHT   = 3
    DIR_LEFT    = 4
    LOOKING_R   = 0.2

    def list_index_trans(self):
        # 返回 [目标位置x, 目标位置y, 守卫ID]
        idx =  [index for index, element in enumerate([self.tracking_list[0],self.tracking_list[3],self.tracking_list[6]]) if element == self.group]
        idx_g =  [index for index, element in enumerate([self.guard_list[0],self.guard_list[2],self.guard_list[4]]) if element == self.group]
        return [self.tracking_list[idx[0]*3 + 1], self.tracking_list[idx[0]*3 + 2], self.guard_list[idx_g[0]*2 + 1]]
    
    def leader_ctrl(self):
        """
        Y
        |
        ----X
        Leader 控制，负责绕圈圈
        往前 直到 y = self.map_l
        往右 10m
        向后 直到 y = 5m
        """
        
        # self.leader_target
        kp_hor=0.3
        ## NED GOTO 方法
        # goto [-8.5, 2]  --> goto [-8.5, 5]  --> goto [-2.0, 5]
        if self.leader_target_index >= len(self.leader_target_poss):
            # formation 结束，leaving
            if self.uav_id == self.guard_id:
                # 自己是guard，离开编队
                self.mission_state = MissionState.MIS_FORMATION_LEAVING
            return [0., 0., 0.]
        self.leader_target = self.leader_target_poss[self.leader_target_index]
        vv_ned = self.goto_pos_NED(self.leader_target[0], self.leader_target[1])
        vv_ned = np.array(vv_ned) / np.linalg.norm(vv_ned) * self.uav_speed
        vv_ned = [vv_ned[0], vv_ned[1], 0.0]
        if self.goto_pose_ready(self.leader_target[0], self.leader_target[1]):
            self.leader_target_index += 1
            self.logger.info(f"[LEADER] target index: {self.leader_target_index};  x: {self.leader_target[0]}, y: {self.leader_target[1]}")
            
        return vv_ned
        vv_frd = [0., 0., 0.]
        if self.local_pos_est is None:
            self.logger.info(f"[LEADER] local pos est is None")
            return vv_frd  # TODO IMPORTANT
        if self.direc == self.DIR_FRONT:
            if self.local_pos_est[1] > -self.map_l:
                vv_frd = [self.uav_speed, 0., 0.]
                vv_frd[1]=kp_hor*(self.local_pos_est[1]-self.leader_target[1])
                
            else:
                # 准备往左，记录现在的x
                
                self.leader_target=[-7,self.left_begin_x + self.dx_per_line,0]###
                
                self.direc = self.DIR_LEFT
                self.left_begin_x = self.local_pos_est[0]
                
                vv_frd = [0., 0., 0.]
        elif self.direc == self.DIR_LEFT:
            if self.local_pos_est[0] < self.left_begin_x + self.dx_per_line:
                # 往左
                vv_frd = [0., -self.uav_speed, 0.]
                vv_frd[0]=kp_hor*(self.local_pos_est[0]-self.leader_target[0])
            else:
                # 往后
                self.leader_target=[2,self.leader_target[1],0]
                self.direc = self.DIR_BACK
                vv_frd = [0., 0., 0.]
        elif self.direc == self.DIR_BACK:
            # 往后知道y=4
            if self.local_pos_est[1] < -2.0:
                vv_frd = [-self.uav_speed, 0., 0.]
                vv_frd[1]=kp_hor*(self.local_pos_est[1]-self.leader_target[1])
            else:
                # 往左
                # self.leader_target=[]
                self.left_begin_x = self.local_pos_est[0]
                self.direc = self.DIR_LEFT
                vv_frd = [0., 0., 0.]
        return vv_frd
    
    def formation_ctrl(self):
        """
        编队控制
        """
        vv_frd = [0.0, 0.0, 0.0]
        if self.formation_role == AgentRole.ROLE_FORMATION_ANGLE_LEADER:
            if not self.color_lost:
                vv_frd = self.form_ctrler.get_ctrl(self.dists, self.color_dx)
            else:
                vv_frd = self.form_ctrler.get_ctrl(self.dists)
        elif self.formation_role == AgentRole.ROLE_FORMATION_LEADER:
            # 调用leader control??还是用类？
            # vv_frd = self.form_ctrler.get_ctrl(self.dists)
            # vv_frd = self.leader_ctrl()
            vv_ned = self.leader_ctrl()
        else:
            # Follower
            vv_frd = self.form_ctrler.get_ctrl(self.dists)
        self.logger.debug(f"[FORM] vv_frd: {vv_frd[0]}, {vv_frd[1]}")
        return [vv_frd[0], vv_frd[1], 0.0]
    
    def formation_leaving(self):
        if self.time_to_leave is None:
            self.get_logger().info("***********i'll leave************")
            self.time_to_leave = time.time()
        vv_ned = [0.,0.,0.]
        if self.local_pos_est is None or time.time() - self.time_to_leave < 0.3:
            return [0., 0., 0.]
        # time.sleep(2.0) # 等其他人先走一走
        # TODO 看具体数字？
        if self.number_detected_state and self.detected_result.detected_num == self.group:
            self.mission_state = MissionState.MIS_TARGET_WATCHING
            vv_ned = [0., 0., 0.]
        else:
            # 位移到已知的目标位置上
            # TODO 记录一下，不要持续更新
            # 坐标系，pose都是ned的，所以可以输出ned的速度
            # vv_ned
            tgt_x = self.tracking_list[self.group * 3 - 2]
            tgt_y = self.tracking_list[self.group * 3 - 1]
            # self.get_logger().info(
            #     f'==========leaving: {tgt_x - self.local_pos_est[0]:.2f},{tgt_y - self.local_pos_est[1]:.2f}============'
            # )
            if abs(tgt_x - self.local_pos_est[0]) > 2:
                vv_ned = [self.uav_speed*((tgt_x - self.local_pos_est[0])/abs(tgt_x - self.local_pos_est[0])), 0., 0.]
            else:
                if abs(tgt_y - self.local_pos_est[1]) > 2:
                    vv_ned = [0., self.uav_speed*((tgt_y - self.local_pos_est[1])/abs(tgt_x - self.local_pos_est[1])), 0.]
                else:
                    self.mission_state = MissionState.LOOKING_AROUND
                    self.local_pos_start = self.local_pos_est
                    self.LOOKING_R = 0.25
                    vv_ned = [0.,0.,0.]
        return vv_ned
    
    LOOKING_AROUND_SPD = 0.5
    def looking_around(self):
        """
        附近瞎逛
        """
        
        if self.number_detected_state:
            self.get_logger().info("***********i can see************")
            self.mission_state = MissionState.MIS_TARGET_WATCHING
            vv_ned = [0.0, 0.0, 0.0]
            return vv_ned
        
        if self.direc == self.DIR_FRONT:
            if self.local_pos_est[1] - self.local_pos_start[1] < self.LOOKING_R:
                vv_ned = [ 0.,self.LOOKING_AROUND_SPD, 0.]
            else:
                self.direc = self.DIR_RIGHT
                vv_ned = [0.0, 0.0, 0.0]
        elif self.direc == self.DIR_RIGHT:
            if self.local_pos_est[0] - self.local_pos_start[0] < self.LOOKING_R:
                vv_ned = [self.LOOKING_AROUND_SPD, 0., 0.]
            else:
                self.direc = self.DIR_BACK
                self.LOOKING_R += 0.25
                vv_ned = [0.0, 0.0, 0.0]
        elif self.direc == self.DIR_BACK:
            if self.local_pos_start[1]  - self.local_pos_est[1] < self.LOOKING_R:
                vv_ned = [ 0.,-self.LOOKING_AROUND_SPD, 0.]
            else:
                self.direc = self.DIR_LEFT
                vv_ned = [0.0, 0.0, 0.0]
        elif self.direc == self.DIR_LEFT:
            if self.local_pos_start[0]  - self.local_pos_est[0] < self.LOOKING_R:
                vv_ned = [-self.LOOKING_AROUND_SPD, 0., 0.]
            else:
                self.direc = self.DIR_FRONT
                vv_ned = [0.0, 0.0, 0.0]
                self.LOOKING_R += 0.25
        return vv_ned

    
    def target_watch_ctrl(self):
        # TODO 只是守着目标，不降落
        # self.get_logger().info(f'=======wathcing for {self.list_index_trans[0]},{self.list_index_trans[1]}=========')
        vv_frd = [0.0, 0.0, 0.0]
        # 如果同组飞机都是降落模式，就允许降落
        should_land = True
        for member in self.member_list:
            if self.uav_infos[member-1].mission_stat != MissionState.MIS_LANDING:
                should_land = False
                break
        if self.number_detected_state:
            if self.detected_result.detected_num == self.group:
                if should_land:
                    self.logger.info(f"===== UAV {self.uav_id} SWITCH TO LAND{MissionState.MIS_LANDING} from watching")
                    self.mission_state = MissionState.MIS_LANDING
                    return vv_frd
                # TODO 确认pose
                # if len(self.member_list)-1 == self.guarded_flag:
                    
                #     # self.get_logger().info(f"detected_pose:{self.detected_result.detected_pose[0], self.detected_result.detected_pose[1]}  ")
                #     vv_frd, final_land_flag = self.land_ctrler.land_ctrl(self.detected_result.detected_pose[0], self.detected_result.detected_pose[1], self.lidar_height)
                #     # 只要开始final_land 之后即便识别到也不管
                #     # self.final_land_flag = self.final_land_flag or final_land_flag
                #     self.logger.debug(f"[LAND] detected num: {self.detected_result.detected_num}, group: {self.group}, vel: {vv_frd}")
                # else:
                vv_frd, final_land_flag = self.land_ctrler.land_ctrl(self.detected_result.detected_pose[0], self.detected_result.detected_pose[1], self.lidar_height)
                # vv_frd = [0.,0.,0.] # TODO 只是守着目标，不降落
                return vv_frd
            else:
                self.logger.debug(f"[LAND] 检测到目标{self.detected_result.detected_num}，但不是目标{self.group}")
                return vv_frd
        else:
            self.logger.info(f"目标检测超时")
            # TODO 附近瞎逛或者离检测到的飞机近一点
            self.mission_state = MissionState.LOOKING_AROUND
            self.local_pos_start = self.local_pos_est
            self.LOOKING_R = 0.4
            # vv_frd = self.looking_around()
            return vv_frd
        
    def land_ctrl(self):
        """
        降落控制
        """
        # 应该先飞到目标位置与高度，然后直接降落，需要一个flag
        vv_frd = [0., 0., 0.]
        if self.lidar_height is not None:
            if self.lidar_height >= 0.2:
                if self.final_land_flag:
                    self.logger.debug(f"[LAND] final land flag is True")
                    vv_frd = [0., 0., 0.25]
                    return vv_frd
                else:
                    if self.number_detected_state:
                        if self.detected_result.detected_num == self.group:
                            # TODO 确认pose
                            # self.get_logger().info(f"detected_pose:{self.detected_result.detected_pose[0], self.detected_result.detected_pose[1]}  ")
                            vv_frd, final_land_flag = self.land_ctrler.land_ctrl(self.detected_result.detected_pose[0], self.detected_result.detected_pose[1], self.lidar_height)
                            # 只要开始final_land 之后即便识别到也不管
                            self.final_land_flag = self.final_land_flag or final_land_flag
                            self.logger.info(f"[LAND] det n: {self.detected_result.detected_num}, gr: {self.group}, vel: {vv_frd}")
                            return vv_frd
                        else:
                            self.logger.info(f"[LAND] 检测到目标{self.detected_result.detected_num}，但不是目标{self.group}")
                            return vv_frd
                    else:
                        self.logger.info(f"未检测到目标，直接降落！")
                        vv_frd = [0, 0, 0.25]
                        # TODO 附近瞎逛或者离检测到的飞机近一点
                        # vv_frd = self.looking_around()
                        return vv_frd
            else:
                return [0,0,1]
        else:
            return vv_frd
    
    def rend_ctrl(self):
        """
        聚合控制
        """
        # 距离远的时候使用之前的估计位置
        # 距离近的时候使用rendezvous方法
        # return [0.0, 0., 0.]
        # TODO 确认id
        vv_ned = [0.0, 0.0, 0.0]
        if self.dists[self.guard_id-1] <= 8:
            self.logger.info(f"====== UAV {self.uav_id} switch to LAND from REND")
            self.mission_state = MissionState.MIS_LANDING
            return vv_ned
        if self.dists[self.guard_id-1] > 8 and not self.arrive_guard_pos:
            # TODO 应该是距离估计比较准之后切换
            # 使用之前的估计位置
            # vv_ned = self.goto_pos_NED(self.list_index_trans()[0], self.list_index_trans()[1], self.filght_height)
            vv_ned = self.goto_pos_NED(self.uav_infos[self.guard_id-1].pos.pose.position.x, 
                                       self.uav_infos[self.guard_id-1].pos.pose.position.y, 
                                       self.filght_height)
            if abs(vv_ned[0]) < 0.05 and abs(vv_ned[1]) < 0.05:
                # 到达目标位置，但距离还是很远
                self.arrive_guard_pos = True
                pass
            # self.logger.info(f"[main] GOTO guard pos: {self.uav_infos[self.guard_id].pos.position.x:.2f}, {self.uav_infos[self.guard_id].pos.position.y:.2f}")
        else:
            # 使用rendezvous方法
            # 或者 looking around
            if self.guard_pos_ned_est is not None:
                vv_ned = self.goto_pos_NED(self.guard_pos_ned_est[0], self.guard_pos_ned_est[1], self.filght_height)
                self.logger.info(f"[main] REND guard pos: {self.guard_pos_ned_est[0]:.2f}, {self.guard_pos_ned_est[1]:.2f}")
            else:
                self.logger.warn("[main] no guard got!")
            pass
        return vv_ned
        pass
    
    def uav_setup(self):
        # 等待初始化与网络连接
        tt = 0
        while self.mission_state == MissionState.MIS_WAIT:
            tt+=1
            if tt % 10 == 0:
                self.logger.info(f"{ColorCodes.green}[MAIN] 等待任务开始{ColorCodes.reset}")
            rclpy.spin_once(self)
            time.sleep(0.2)
        if self.mission_state == MissionState.MIS_READY:
            # TODO 解锁起飞
            self.arm_cmd_pub.publish(Bool(data=True))
            self.logger.info("[MAIN] 解锁")
            time.sleep(1)
            # 起飞
            self.logger.info("[MAIN] 起飞")
            pass
        elif self.mission_state == MissionState.MIS_BEGIN:
            self.logger.info(f"{ColorCodes.magenta}[MAIN] 等待任务开始{ColorCodes.reset}")
            # 开始任务
            return True
        pass
        
    def run(self):
        # 初始化无人机
        self.uav_setup()
        
        # 执行任务
        while True:
            self.logger.info(f"{ColorCodes.red}[main] DIS NOT GOT{ColorCodes.reset}")
            rclpy.spin_once(self)
            if self.dist_got:
                break
            else:
                time.sleep(0.5)
        r_hat = np.array([5, 0])
        vv = r_hat / np.linalg.norm(r_hat)
        MAX_SIZE = 3000
        TIME_STEP = 0.1
        MAX_SPD = 0.43
        MIN_SPD = 0.02
        MIN_DIST = 1.1
        self.vel_pub_count = 0
        # self.mission_state = MissionState.MIS_LANDING
        self.mission_state = MissionState.MIS_FORMATION
        # self.mission_state = MissionState.MIS_WAIT
        self.logger.info(f"{ColorCodes.green}[MAIN] 主线程开始{ColorCodes.reset}")
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                vv_frd = [0.0, 0.0, 0.0]
                vv_ned = None
                force_frd = False
                
                ####### 测试 #######
                ## rend 
                ## 测试方法：id1 为 id2 的guard，并置位
                # if True:
                #     self.mission_state = MissionState.MIS_GOTO_GUARD
                #     self.logger.info(f"[TESTING] [REND] guard_id: {self.guard_id}@{self.uav_infos[self.guard_id-1].pos.pose.position.x:6.2f}{self.uav_infos[self.guard_id-1].pos.pose.position.y:6.2f}. I'm @ {self.local_pos_est[0]:6.2f}, {self.local_pos_est[1]:6.2f}")
                
                ## Formation only
                ## 测试方法：第一台飞机手飞或程控，前往指定地点
                ## 第二台飞机控制方向
                ## 其余飞机组成编队
                
                if True:
                    self.mission_state = MissionState.MIS_FORMATION
                    # self.logger.info(f"[TESTing] [FROMATION] vvfrd:{}")
                ####### 测试 END #######
                
                
                # 根据状态执行任务
                if self.mission_state == MissionState.MIS_WAIT:
                    # 收到地面站指令之后进入formation
                    pass
                elif self.mission_state == MissionState.MIS_FORMATION:
                    # 收到地面站指令
                    vv_frd = self.formation_ctrl()
                    self.logger.info(f"formation frd{vv_frd[0]:5.2f}, {vv_frd[1]:5.2f}")
                    if self.formation_role == AgentRole.ROLE_FORMATION_LEADER:
                        # force_frd = True
                        pass
                    if self.formation_role == AgentRole.ROLE_FORMATION_LEADER:
                        vv_ned = self.leader_ctrl()
                    pass
                elif self.mission_state == MissionState.MIS_FORMATION_LEAVING:
                    # 进入：自己是该走的那一个，并且自己那一组的目标被发现了
                    # 离开：当自己的目标被发现了
                    # vv_frd = self.formation_leaving()
                    vv_ned = self.formation_leaving()
                    pass
                elif self.mission_state == MissionState.LOOKING_AROUND:
                    # 要去找目标，但去了位置没找到，附近找找
                    # 直到找到目标，进入MIS_TARGET_WATCHING状态
                    vv_ned = self.looking_around()
                    pass
                elif self.mission_state == MissionState.MIS_GOTO_GUARD:
                    # 进入：当任务时间到达/任务结束/所有目标找到时
                    # 离开：本组目标距离较近时，准备降落（离guard比较近的时候）
                    vv_ned = self.rend_ctrl()
                    
                # elif self.mission_state == MissionState.MIS_BASE_MOVE:
                    # pass
                elif self.mission_state == MissionState.MIS_TARGET_WATCHING:
                    # 进入：对于坚守者，找到目标之后，在本地坚守等待
                    # 离开：本组目标都到达附近
                    vv_frd = self.target_watch_ctrl()
                    pass
                elif self.mission_state == MissionState.MIS_LANDING:
                    # 进入：对于坚守者，队友都降落了；对于其他人，离坚守者不足1m或者1.5m的时候
                    # 离开：降落之后，上锁
                    vv_frd = self.land_ctrl()
                    pass
                
                if vv_ned is not None:
                    # 使用NED坐标系
                    vv = vv_ned if isinstance(vv_ned, np.ndarray) else np.array(vv_ned)
                    vv = vv if np.linalg.norm(vv) < MAX_SPD else vv*MAX_SPD/np.linalg.norm(vv)
                    vv[0] = self.vel_ned_set_x_filter.get_value(vv[0])
                    vv[1] = self.vel_ned_set_y_filter.get_value(vv[1])
                    #定高
                    if self.lidar_height is not None and self.mission_state != MissionState.MIS_LANDING:
                        vv[2] = self.height_ctrler.get_ctrl(self.lidar_height,self.rpy)
                    vel_ned_msg = TwistStamped()
                    vel_ned_msg.twist.linear.x = vv[0] if math.fabs(vv[0]) > MIN_SPD else 0.0
                    vel_ned_msg.twist.linear.y = vv[1] if math.fabs(vv[1]) > MIN_SPD else 0.0
                    vel_ned_msg.twist.linear.z = vv[2] 
                    self.vel_ned_pub.publish(vel_ned_msg)
                    self.vel_pub_count += 1
                    if self.vel_pub_count % 15 == 0:
                        self.logger.info(
                            f"{ColorCodes.cyan} mis {self.mission_state} NED: {vel_ned_msg.twist.linear.x :4.2f}, {vel_ned_msg.twist.linear.y:4.2f}, {vv[2]:4.2f}; x {self.local_pos_est[0]:4.1f}, y {self.local_pos_est[1]:4.1f}{ColorCodes.reset}"
                        )
                        self.logger.info(f"dist: {self.dists}")
                    time.sleep(0.05)
                    continue
                    pass
                # 速度输出
                vv_frd = [vv_frd[0], vv_frd[1], 0.0] if len(vv_frd)==2 else vv_frd
                vv = vv_frd if isinstance(vv_frd, np.ndarray) else np.array(vv_frd)
                vv = vv if np.linalg.norm(vv) < MAX_SPD else vv*MAX_SPD/np.linalg.norm(vv)
                vv[0] = self.vel_frd_set_x_filter.get_value(vv[0])
                vv[1] = self.vel_frd_set_y_filter.get_value(vv[1])
                vv[2] = vv[2]
                vel_type = 2  # 1:ned, 2:frd
                
                # frd to ned
                if np.linalg.norm(self.rpy)>0 and not force_frd:# 还没收到imu就不改了
                    vel_type = 1 # ned
                    yaw = self.rpy[0]
                    # self.logger.info(f"=============rpy: {self.rpy[0]},  {self.rpy[1]},  {self.rpy[2]}")
                    # TODO 后面变成ned了
                    vv = np.array([
                        vv[0]*math.cos(yaw) - vv[1]*math.sin(yaw),
                        vv[0]*math.sin(yaw) + vv[1]*math.cos(yaw),
                        vv[2]
                    ])
                if self.mission_state == MissionState.MIS_FORMATION:
                    if np.linalg.norm(self.leader_vel_ned)>0:
                        vv = vv + np.array(self.leader_vel_ned)
                
                # vv[2]=0.0
                if self.lidar_height is not None and self.mission_state != MissionState.MIS_LANDING:
                    vv[2] = self.height_ctrler.get_ctrl(self.lidar_height,self.rpy)
                #     # self.get_logger().info(
                #     #     f"vz:{vv[2]}  "
                #     # )
                
                vel_frd_msg = TwistStamped()
                vel_frd_msg.twist.linear.x = vv[0] if math.fabs(vv[0]) > MIN_SPD else 0.0
                vel_frd_msg.twist.linear.y = vv[1] if math.fabs(vv[1]) > MIN_SPD else 0.0
                vel_frd_msg.twist.linear.z = vv[2] 
                
                # self.get_logger().info(
                #     f"d:{self.dists[0] :6.2f} {self.dists[1] :6.2f} {self.dists[2] :6.2f} {self.dists[3] :6.2f} == " + 
                #     f"v frd: {vv[0]:4.2f}, {vv[1]:4.2f}, {vv[2]:4.2f} ; h:{self.lidar_height} "
                # )
                if vel_type == 1 and not force_frd: # NED
                    self.vel_ned_pub.publish(vel_frd_msg)
                    self.vel_pub_count += 1
                    if self.vel_pub_count % 15 == 0:
                        self.logger.info(
                            f"{ColorCodes.cyan} mis {self.mission_state} vNED: {vel_frd_msg.twist.linear.x :4.2f}, {vel_frd_msg.twist.linear.y:4.2f}, {vv[2]:4.2f}; x {self.local_pos_est[0]:4.1f}, y {self.local_pos_est[1]:4.1f}{ColorCodes.reset}"
                        )
                        self.logger.info(f"dist: {self.dists}")
                else:
                    self.vel_frd_pub.publish(vel_frd_msg)
                    self.vel_pub_count += 1
                    if self.vel_pub_count % 15 == 0:
                        self.logger.info(
                            f"{ColorCodes.green} mis {self.mission_state} vFRD: {vel_frd_msg.twist.linear.x :4.2f}, {vel_frd_msg.twist.linear.y:4.2f}, {vv[2]:4.2f}; x {self.local_pos_est[0]:4.1f}, y {self.local_pos_est[1]:4.1f}{ColorCodes.reset}"
                        )
                        self.logger.info(f"dist: {self.dists}")
                # rclpy.spin_once(self)
                
                time.sleep(0.05)
            except Exception as e:
                self.logger.info(f"[MAIN] Exception: {e.with_traceback()}")
                vel_frd_msg = TwistStamped()
                self.vel_frd_pub.publish(vel_frd_msg)
                
                time.sleep(0.05)
        self.logger.info(f"{ColorCodes.red}[MAIN] 主线程结束{ColorCodes.reset}")
        

def main(args=None):
    rclpy.init()
    
    formation_node = Comp29MainNode(args)
    
    try:
        formation_node.run()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
