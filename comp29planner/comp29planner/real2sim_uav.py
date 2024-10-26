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
from std_msgs.msg           import Float32MultiArray, Header
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped, Vector3
from std_msgs.msg           import Int64, Float64, Bool, Float32
from comp29msg.msg          import CommunicationInfo, DetectionResult, DetectionArray, UAVInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from sensor_msgs.msg import Imu

from px4_msgs.msg import VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np

uav_num = 8
uav_xyz_yaw = [ [0, 0, 0, 0] for i in range(uav_num) ]
uav_v_xyz   = [ [0, 0, 0   ] for i in range(uav_num) ]

def mat2ros_mat(matrix):
    # 创建Float32MultiArray消息
    matrix_msg = Float32MultiArray()

    # 设置矩阵维度信息（假设是n*n矩阵）
    dim = MultiArrayDimension()
    dim.label = "height"
    dim.size = len(matrix)
    dim.stride = len(matrix) * len(matrix[0])
    matrix_msg.layout.dim.append(dim)

    dim = MultiArrayDimension()
    dim.label = "width"
    dim.size = len(matrix[0])
    dim.stride = len(matrix[0])
    matrix_msg.layout.dim.append(dim)

    # 将矩阵数据转换为一维列表并赋值给消息的data字段
    matrix_msg.data = [item for row in matrix for item in row]
    
    return matrix_msg

def cal_dist_mat():
    dist_mat = [[0 for i in range(uav_num+1)] for j in range(uav_num+1)]
    for i in range(uav_num):
        for j in range(i+1, uav_num):
            dist_mat[i+1][j+1] = np.linalg.norm(np.array(uav_xyz_yaw[i][:3]) - np.array(uav_xyz_yaw[j][:3]))
            dist_mat[j+1][i+1] = dist_mat[i][j]
    return dist_mat

class Env(Node):
    def __init__(self, 
                target_pos=[(1, 1), (10, 10), (20, 20)],
                UAV_num = 4,
                noise_level = 0.1,
                ) -> None:
        super().__init__('sim_env_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
        )
        self.logger = self.get_logger()
        self.dist_mat_pubs = [
            self.create_publisher(Float32MultiArray, f'/uav_{i+1}/uwb_filtered', qos_profile) for i in range(UAV_num)
        ]
        self.detec_rst_pubs = [
            self.create_publisher(DetectionResult, f'/uav_{i+1}/detection_result', qos_profile) for i in range(UAV_num)
        ]
        self.color_offset_pub = self.create_publisher(Vector3, '/uav_2/color_offset', qos_profile)
        self.create_timer(0.1, self.timer_callback)
        
            ## XY像素大小
        self.PIX_X_MAX = 640
        self.PIX_Y_MAX = 480
        ## XY视角大小
        self.X_ANGLE = 85  
        self.Y_ANGLE = 60
    
    def timer_callback(self):
        dist_msg = mat2ros_mat(cal_dist_mat())
        self.dist_mat_pubs.publish(dist_msg)
        
    def pos2pix(self, pos_from, pos_to, yaw, orien ):
        ## orien in ['f', 'd']
        pos_from = np.array(pos_from)
        pos_to = np.array(pos_to)
        related_pos =  pos_to - pos_from
        if orien == 'f':
            ## 向前看，如果能看到，返回像素坐标，否则返回-1，-1
            if abs(np.arctan(related_pos[1]/related_pos[0]) - yaw) < self.X_ANGLE or np.arctan(related_pos[2]/related_pos[0])> self.Y_ANGLE:
                pix_x = -1
                pix_y = -1
            else:
                [pix_x, pix_y]= self.cal_detect_result(related_pos,orien,yaw)
            pass
        elif orien == 'd':
            ## 向下看，如果能看到，返回像素坐标，否则返回-1，-1
            if np.arctan(abs(related_pos[0])/related_pos[2])> self.Y_ANGLE or np.arctan(abs(related_pos[1])/related_pos[2])> self.X_ANGLE:
                pix_x = -1
                pix_y = -1
            else:
                [pix_x, pix_y] = self.cal_detect_result(related_pos,orien,yaw)
            pass
        pass
    def cal_detect_result(self, related_pos, orien,yaw):
        ## 计算能看到的像素坐标
        X_base = self.PIX_X_MAX/2/np.tan(self.X_ANGLE)
        Y_base = self.PIX_Y_MAX/2/np.tan(self.Y_ANGLE)
        if orien == 'f':
            pix_x = self.PIX_X_MAX/2 + (abs(related_pos.pos[1])/related_pos[1]) * X_base*np.tan(abs(related_pos[1])/related_pos[0])
            pix_y = self.PIX_Y_MAX/2 + (abs(related_pos.pos[2])/related_pos[2]) * Y_base*np.tan(abs(related_pos[2])/related_pos[0])
        elif orien =='d':
            pix_x = self.PIX_X_MAX/2 + (abs(related_pos.pos[1])/related_pos[1]) * X_base*np.tan(abs(related_pos[1])/related_pos[2])
            pix_y = self.PIX_Y_MAX/2 + (abs(related_pos.pos[0])/related_pos[0]) * Y_base*np.tan(abs(related_pos[0])/related_pos[2])
        return [pix_x, pix_y]

class UavInfoTransNode(Node):
    def __init__(self, 
                uav_id = 8
                ) -> None:
        super().__init__('sim_env_node')
        self.logger = self.get_logger()
        self.uav_id = uav_id
        self.lidar_height = 0
        self.angular_vel = [0,0,0]
        self.quat =None
        # 消息控制
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
        )
        # sim2planner
        self.local_position_ned_pub     = self.create_publisher(PoseStamped, 
                                            f'uav_{uav_id}'+'/local_position_ned', 
                                            qos_profile) 
        
        self.velocity_angular_pub       =  self.create_publisher(TwistStamped, 
                                            f'uav_{uav_id}'+'/velocity_and_angular', 
                                            qos_profile) 
        
        self.lidar_height_pub           = self.create_publisher(Float64, 
                                            f'uav_{uav_id}'+'/lidar_height', 
                                            qos_profile) 
        
        self.super_mission_state_pub    = self.create_publisher(Int64, 
                                            f'uav_{uav_id}'+'/super_mission_state', 
                                            qos_profile) 
        
        ## subscriber
        self.uav_pose_sim_sub          = self.create_subscription(
                VehicleLocalPosition,
                f'px4_{uav_id}/fmu/out/vehicle_local_position',
                self.UavPoseCb,
                qos_profile)
        
        self.create_timer(0.1, self.timer_callback)

    def UavPoseCb(self, msg):
        uav_xyz_yaw[self.uav_id - 1][0] = msg.x
        uav_xyz_yaw[self.uav_id - 1][1] = msg.y
        uav_xyz_yaw[self.uav_id - 1][2] = msg.z
        
        self.lidar_height = -msg.z
        
        uav_v_xyz[self.uav_id - 1][0]   = msg.vx
        uav_v_xyz[self.uav_id - 1][1]   = msg.vy
        uav_v_xyz[self.uav_id - 1][2]   = msg.vz
        
        pass

    def odomCb(self, msg):
        self.angular_vel[0] = msg.angular_velocity[0]
        self.angular_vel[1] = msg.angular_velocity[1]
        self.angular_vel[2] = msg.angular_velocity[2]
        
    pass
    def get_msg_header(self):
        header = Header()
        header.frame_id = 'world'
        header.stamp = self.get_clock().now().to_msg()
        return header

    def timer_callback(self):
        ## 发布信息
        local_position_ned_msg = PoseStamped()
        velocity_angular_msg = TwistStamped()
        lidar_height_msg = Float64()
        super_mission_state_msg = Int64()
        
        local_position_ned_msg.pose.position.x = uav_xyz_yaw[self.uav_id - 1][0]
        local_position_ned_msg.pose.position.y = uav_xyz_yaw[self.uav_id - 1][1]
        local_position_ned_msg.pose.position.z = uav_xyz_yaw[self.uav_id - 1][2]
        local_position_ned_msg.header = self.get_msg_header()
        self.local_position_ned_pub.publish(local_position_ned_msg)
        
        velocity_angular_msg.twist.linear.x = uav_v_xyz[self.uav_id - 1][0]
        velocity_angular_msg.twist.linear.y = uav_v_xyz[self.uav_id - 1][1]
        velocity_angular_msg.twist.linear.z = uav_v_xyz[self.uav_id - 1][2]
        velocity_angular_msg.twist.angular.x = self.angular_vel[0]
        velocity_angular_msg.twist.angular.y = self.angular_vel[1]
        velocity_angular_msg.twist.angular.z = self.angular_vel[2]
        velocity_angular_msg.header = self.get_msg_header()
        self.velocity_angular_pub.publish(velocity_angular_msg)
        
        lidar_height_msg.data = self.lidar_height
        self.lidar_height_pub.publish(lidar_height_msg)

        super_mission_state_msg.data = 0
        self.super_mission_state_pub.publish(super_mission_state_msg)

def spin_t(node, rate = 10):
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(1/rate)
    

def main(arg=None)->None:
    rclpy.init()
    env = Env(UAV_num=uav_num)
    uavs = []
    for i in range(uav_num):
        uavs.append(UavInfoTransNode(uav_id=i+1))
    import threading
    env_t = threading.Thread(target=spin_t, args=(env,))
    env_t.start()
    uav_ts = []
    for uav in uavs:
        uav_ts.append(threading.Thread(target=spin_t, args=(uav,)))
        uav_ts[-1].start()


