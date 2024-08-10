#!/usr/bin/python3
import math
import socket
import threading
import time
from pymavlink import mavutil
import serial
import warnings
from .config import *
import os
import logging
import json


class UAVCfg:
    def __init__(self, cfg_file="config.json") -> None:
        if cfg_file.endswith("py"):
            # use python config file
            logging.info("use PYTHON config file")
            from config import UAV_cfg
            self.config = UAV_cfg
        elif cfg_file.endswith("json"):
            # use json config file
            logging.info("use JSON config file")
            self.config = self.load_config(cfg_file)
        pass
    
    def get_config(self):
        return self.config
    
    def load_config(self, file_name):
        with open(file_name, "r") as f:
            config = json.load(f)
        return config


class Pose:
    def __init__(self, x=0, y=0, z=0, r=0, p=0, yaw=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.p = p
        self.yaw = yaw
        pass
    
        
class UAVOnBoard:
    POS_SRC_NONE  = -1
    POS_SRC_VICON = 0
    POS_SRC_LOCAL = 1
    POS_SRC_GPS   = 2
    POS_SRC_CALCULATED = 3
    
    FCU_TYPE_PX4 = 0
    FCU_TYPE_ANO = 1
    
    FCU_TYPE_DICT = {
        "PX4": FCU_TYPE_PX4,
        "ANO": FCU_TYPE_ANO
    }
    
    VEL_MODE_NED = 1
    VEL_MODE_FRD = 2
    def __init__(self, 
                config_file = "config.json",
                ros_on = 0, # 0 for off; 1 for ros1; 2 for ros2
                ) -> None:
        self.pose           = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.vel            = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.vel_mode       = self.VEL_MODE_NED
        self.vel_set_frd    = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.vel_set_ned    = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.group = 0
        self.detect_result = DetectResult()
        self.pos_source = self.POS_SRC_NONE # Position source
        self.mission_state = DrvCmd.DRV_CMD_TASK_WAIT
        # flags
        self.pos_got_ever = False
        self.takeoff_confirm_without_position = False
        self.is_armed = False
        self.ctrl_block_flag = True  # 用于其他任务暂停起飞和降落的标志, True: 可以持续运行，False: 立刻截止当前任务
        # connection
        self.fcu_master = None
        self.gcs_master_udpout = None
        self.gcs_master_udpin = None
        self.arm_height = 0
        self.backup1_flag = False
        self.backup2_flag = False
        self.est_flag = True
        self.battery        = None
        self.lidar_height   = None
        self.imu_acc        = None
        self.imu_gyro       = None
        self.odom_xyz       = None
        self.odom_q         = None
        self.odom_v         = None
        self.servos         = None
        self.time_sync      = None
        self.vfr_hud        = None
        self.opti_xyq       = None
        self.gps            = None
        # config
        if config_file is None:
            pass
            # self.gcs_protocol = uav_config["gcs_protocol"]
            # self.gcs_config = uav_config["gcs_config"]
            # self.fcu_config = uav_config["fcu_config"]
        else:
            if isinstance(config_file, str):
                if not os.path.exists(config_file):
                    warnings.warn("配置文件不存在，请检查配置文件路径\nConfig file not found, please check the path")
                self.uav_cfg = UAVCfg(config_file).get_config()
            if isinstance(config_file, dict):
                self.uav_cfg = config_file
            self.gcs_protocol = self.uav_cfg['CONNECTION']['GCS_CONNECTION']["GCS_PROTOCOL"]
            print(self.gcs_protocol)
            self.gcs_config = self.uav_cfg['CONNECTION']['GCS_CONNECTION']
            self.fcu_config = self.uav_cfg['CONNECTION']['FCU_CONNECTION']
            self.video_on = self.gcs_config["VIDEO_ON"]
            if self.video_on:
                self.video_protocol = self.gcs_config["VIDEO_PROTOCOL"]
                self.video_ip = self.gcs_config["VIDEO_IP"]
                self.video_port = self.gcs_config["VIDEO_PORT"]
        
        if not self.uav_cfg["FCU_TYPE"] in self.FCU_TYPE_DICT.keys():
            warnings.warn("飞控类型错误，请检查配置文件，默认设置为PX4\nFCU type error, please check the config file. PX4 is set as default")
            self.fcu_type = self.FCU_TYPE_PX4
        else:
            self.fcu_type = self.FCU_TYPE_DICT[self.uav_cfg["FCU_TYPE"]]
        
        self.fcu_port = self.fcu_config["FCU_PORTS"]
        self.fcu_baud = self.fcu_config["FCU_BAUD"]
        if self.gcs_protocol == "udp":
            self.gcs_ip = self.gcs_config["GCS_IP"]
            self.local_ip = self.gcs_config["LOCAL_IP"]
            self.msg_in_port_gcs = self.gcs_config["MSG_IN_PORT_GCS"]
            self.msg_in_port_local = self.gcs_config["MSG_IN_PORT_LOCAL"]
        elif self.gcs_protocol == "uart":
            self.gcs_uart_port = self.gcs_config["GCS_UART_PORT"]
            self.gcs_uart_baud = self.gcs_config["GCS_UART_BAUD"]
        if self.video_on:
            self.video_ip = self.gcs_ip if self.video_ip is None else self.video_ip
        self.open_flag = True
        # init
        self.onboard_mavlink_init()
        self.should_shutdown = False
        self.ros_on = ros_on
        if self.open_flag:
            print("连接初始化完成")
            self.gcs_master_udpin.port.settimeout(1)
            self.fcu_master.port.timeout = 1
            self.data_trans_begin()
            if self.ros_on:
                self.ros_init()
        pass
        self.lidar_height = 0
        
    def ros_init(self):
        if self.ros_on == 1:
            pass
        elif self.ros_on == 2:
            self.ros2_init()
        pass
    
    def ros2_init(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped, PoseStamped
        from quaternions import Quaternion as Quaternion
        from std_msgs.msg import Float32MultiArray
        from sensor_msgs.msg import Imu
        from std_srvs.srv import Trigger
        self.fcu_node = Node('fcu_publisher')
        self.fcu_logger = rclpy.logging.get_logger("record")
        self.imu_pub             = self.node.create_publisher(Imu, 'imu', 10)
        self.servo_pub           = self.node.create_publisher(Float32MultiArray, 'servo_data', 1)
        self.attitude_tgt_pub    = self.node.create_publisher(Float32MultiArray, 'attitude_setpt', 1)
        self.local_pos_pub       = self.node.create_publisher(PoseStamped,       'local_position_ned', 1)
        self.gps_pos_pub         = self.node.create_publisher(PoseStamped,       'gps_position', 1)
        self.velocity_pub        = self.node.create_publisher(TwistStamped, 'velocity_and_angular', 1)
        
        self.fcu_ctrl_sub        = self.node.create_subscription(TwistStamped, 'fcu_ctrl', self.fcu_ctrl_callback, 10)
        self.srv_arm            = self.node.create_service(Trigger, 'arm', self.arm_callback)
        self.srv_takeoff        = self.node.create_service(Trigger, 'takeoff', self.takeoff_callback)
        self.srv_land           = self.node.create_service(Trigger, 'land', self.land_callback)
        self.get_logger().info('Drone Command Server is ready.')
        
    def arm_callback(self, request, response):
        self.get_logger().info('Received command: ARMING')
        # 这里添加ARMING逻辑
        self.arm_uav()
        response.success = True
        response.message = 'Arming successful'
        return response

    def takeoff_callback(self, request, response):
        self.get_logger().info('Received command: TAKEOFF')
        # 这里添加TAKEOFF逻辑
        response.success = True
        response.message = 'Takeoff successful'
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Received command: LANDING')
        # 这里添加LANDING逻辑
        response.success = True
        response.message = 'Landing successful'
        return response

    
    def onboard_mavlink_init(self):
        ## init mavlink for fcu and gcs connection
        ######################## fcu connection ########################
        try:
            self.fcu_master = mavutil.mavlink_connection(self.fcu_port, baud=self.fcu_baud)
            self.fcu_master.mav.version = 'v2.0'
            print("飞控连接成功@" + self.fcu_port + " " + str(self.fcu_baud))
            print("FCU connected@" + self.fcu_port + " " + str(self.fcu_baud))
        except Exception as e:
            print(e)
            self.open_flag = False
            warnings.warn("打开串口出现错误，无法连接飞控，请检查串口顺序，并使能串口\n Error in connecting to fcu, please check the port and baudrate")
            
        ######################## gcs connection ########################
        if self.gcs_protocol == "uart":
            # TODO uart not tested
            # self.open_flag = False
            warnings.warn("串口模式未测试\n uart mode not tested")
            try:
                self.gcs_ser = serial.Serial(self.gcs_uart_port, baudrate=self.gcs_uart_baud)
                self.gcs_master_udpout = mavutil.mavlink_connection(self.gcs_ser,source_system=255, source_component=1)
                self.gcs_master_udpout.mav.version = 'v2.0'
            except Exception:
                self.open_flag = False
                warnings.warn("打开串口出现错误，无法连接飞控或地面站，请检查串口顺序，并使能串口\n Error in connecting to fcu or gcs, please check the port and baudrate")
        elif self.gcs_protocol == "udp":
            try:
                self.gcs_master_udpout = mavutil.mavlink_connection(f"udpout:{self.gcs_ip}:{self.msg_in_port_gcs}",source_system=255, source_component=1)
                self.gcs_master_udpout.mav.version = 'v2.0'
                print("打开udpout网口成功@" + f"udpout:{self.gcs_ip}:{self.msg_in_port_gcs}")
                print("GCS connected@" + f"udpout:{self.gcs_ip}:{self.msg_in_port_gcs}")
            except Exception as e:
                print(e)
                self.open_flag = False
                warnings.warn("打开udpout网口出现错误，无法连接地面站，请检查串口顺序，并使能串口\n Error in connecting to fcu or gcs, please check the port and baudrate")
                
            try:
                self.gcs_master_udpin = mavutil.mavlink_connection(f"udpin:{self.local_ip}:{self.msg_in_port_local}",source_system=255, source_component=1)
                self.gcs_master_udpin.mav.version = 'v2.0'
                print("打开udpin网口成功@" + f"udpin:{self.local_ip}:{self.msg_in_port_local}")
            except Exception as e:
                print(e)
                self.open_flag = False
                warnings.warn("打开udpin网口出现错误，无法连接地面站，请检查串口顺序，并使能串口\n Error in connecting to fcu or gcs, please check the port and baudrate") 
    
    def data_trans_begin(self):
        t1 = threading.Thread(target = self.data_gcs2computer)
        t2 = threading.Thread(target = self.data_fcu2gcs_t)
        t_heart = threading.Thread(target = self.heartbeat_t)
        t1.start()
        t2.start()
        t_heart.start()
        
    def set_pose(self, pos):
        self.pose = pos
        
    def set_vel(self, vel):
        self.vel = vel

    def heartbeat_t(self):
        system_type = mavutil.mavlink.MAV_TYPE_QUADROTOR
        autopilot_type = mavutil.mavlink.MAV_AUTOPILOT_GENERIC
        base_mode = 0
        custom_mode = 0
        while not self.should_shutdown:
            self.fcu_master.mav.heartbeat_send(
                type=system_type,
                autopilot=autopilot_type,
                base_mode=base_mode,
                custom_mode=custom_mode,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE
            )
            self.gcs_master_udpout.mav.heartbeat_send(
                type=system_type,
                autopilot=autopilot_type,
                base_mode=base_mode,
                custom_mode=custom_mode,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE
            )
            time.sleep(0.3)
    
    def start_send_vel_cmd_t(self):
        t = threading.Thread(target=self.send_vel_cmd_t)
        t.start()
        
    def send_vel_cmd_t(self):
        tt = 0
        while not self.should_shutdown:
            if self.vel_mode == self.VEL_MODE_FRD:
                self.uav_send_speed_FRD(self.vel_set_frd.x, self.vel_set_frd.y, self.vel_set_frd.z)
                tt += 1
                if tt % 10 == 0:
                    pass
                    # print(f"send vel frd: vx: {self.vel_set_frd.x},  vy: {self.vel_set_frd.y} vz: {self.vel_set_frd.z}")
                    # print(f"spd:          vx: {self.vel.x:.2f}, vy: {self.vel.y:.2f}, vz: {self.vel.z:.2f}")
                    # print(f"height:       h:  {self.lidar_height/100:.2f}")
                    # print(f"pose:         px: {self.pose.x:.2f}, py: {self.pose.y:.2f}, pz: {self.pose.z:.2f}")
                    # print("-------------------------------------------------------------")
            if self.vel_mode == self.VEL_MODE_NED:
                self.uav_send_speed_ned(self.vel_set_ned.x, self.vel_set_ned.y, self.vel_set_ned.z)
                tt += 1
                if tt % 10 == 0:
                    pass
                    # print(f"send vel ned: vx: {self.vel_set_ned.x},  vy: {self.vel_set_ned.y} vz: {self.vel_set_ned.z}")
                    # print(f"spd:          vx: {self.vel.x:.2f}, vy: {self.vel.y:.2f}, vz: {self.vel.z:.2f}")
                    # print(f"height:       h:  {self.lidar_height/100:.2f}")
                    # print(f"pose:         px: {self.pose.x:.2f}, py: {self.pose.y:.2f}, pz: {self.pose.z:.2f}")
                    # print("-------------------------------------------------------------")
            time.sleep(0.05)
    
    def send_offboard_command(self):
        for _ in range(15):
            self.uav_send_speed_FLU(0, 0, 0)
            time.sleep(0.1)
    def uav_send_pose_ned(self, x, y, z, yaw_degree = 0):
        type_mask = 0b0000111111111000  # 忽略位置信息，只设置速度
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用局部NED坐标系
            type_mask,
            x, y, z,  # 目标位置（忽略，因为我们设置了速度）
            0, 0, 0,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            yaw_degree * 3.1415 / 180., 0  # 目标偏航角度和偏航速率（忽略）
        )
        
    def uav_send_speed_ned(self, spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec, yaw_degree=0):
        type_mask = 0b0000111111000111  # 忽略位置信息，只设置速度
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用局部NED坐标系
            type_mask,
            0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
            spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            yaw_degree * 3.1415 / 180., 0  # 目标偏航角度和偏航速率（忽略）
        )

    def uav_send_speed_FLU(self, spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec, yaw_degree=0):
        # type_mask = 0b0000111111000111  # 忽略位置信息，只设置速度
        # self.fcu_master.mav.set_position_target_local_ned_send(
        #     0,  # 时间戳，0 表示立即执行
        #     self.fcu_master.target_system, self.fcu_master.target_component,
        #     mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
        #     type_mask,
        #     0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        #     spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec,  # 目标速度
        #     0, 0, 0,  # 目标加速度（忽略）
        #     yaw_degree * 3.1415 / 180., 0  # 目标偏航角度和偏航速率（忽略）
        # )
        self.uav_send_speed_FRD(spd_x_m_per_sec, -spd_y_m_per_sec, -spd_z_m_per_sec, yaw_degree)   

    def uav_send_speed_FRD(self, spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec, yaw_degree=0):
        type_mask = 0b0000111111000111
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            # mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用FRD坐标系
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # 使用FRD坐标系
            type_mask,
            0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
            spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            yaw_degree * 3.1415 / 180., 0  # 目标偏航角度和偏航速率（忽略）
        )

    def uav_turn_yaw_rad(self, rad):
        # 北零 左逆负，右顺正 -pi -- pi
        type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
            type_mask,
            0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
            0, 0, 0,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            rad, 0  # 目标偏航角度和偏航速率（忽略）
        )
        
    def uav_turn_yaw_speed_rad(self, speed_rad):
        # 北零 左逆负，右顺正 -pi -- pi
        type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
            type_mask,
            0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
            0, 0, 0,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            0, speed_rad  # 目标偏航角度和偏航速率（忽略）
        )
        
    def uav_turn_yaw_angle(self, angle):
        type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
        self.fcu_master.mav.set_position_target_local_ned_send(
            0,  # 时间戳，0 表示立即执行
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
            type_mask,
            0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
            0, 0, 0,  # 目标速度
            0, 0, 0,  # 目标加速度（忽略）
            angle/57.3, 0  # 目标偏航角度和偏航速率（忽略）
        )

    def uav_hover(self):
        self.uav_send_speed_FLU(0,0,0)

    def uav_land_opened(self):
        self.vel_set_frd.x = 0
        self.vel_set_frd.y = 0
        self.vel_set_frd.z = 0.2
        
    def uav_land(self):
        self.fcu_master.mav.command_long_send(
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            1,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        # for i in range(100):
        #     if not self.ctrl_block_flag:
        #         print("[INFO] LAND process stopped")
        #         break
        #     self.uav_send_speed_FLU(0,0,-0.2)
        #     print("landing")
        #     time.sleep(0.1)

    def uav_take_off(self, takeoff_height = 0.6):
        self.fcu_master.mav.command_long_send(
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            1, 0, 0, 0, 0, 0, -takeoff_height 
        )
        # for i in range(45):
        #     if not self.ctrl_block_flag:
        #         print("[INFO] takeoff process stopped")
        #         break
        #     self.uav_send_speed_FLU(0,0,0.25)
        #     print("taking off")
        #     time.sleep(0.1)
        # self.uav_hover()

    def uav_takeoff_opened(self, height = 0.9):
        TAKEOFF_SPD = 0.2
        self.vel_set_frd.x = 0
        self.vel_set_frd.y = 0
        self.vel_set_frd.z = -TAKEOFF_SPD
        try:
            for i in range(int(height / TAKEOFF_SPD * 5)):
                if not self.ctrl_block_flag:
                    print("[INFO] takeoff process stopped")
                    break
                print("taking off")
                time.sleep(0.2)
        except Exception:
            print("[ERROR] takeoff error")
        self.vel_set_frd.x = 0.0
        self.vel_set_frd.y = 0.0
        self.vel_set_frd.z = 0.0
            
    def uav_takeoff_closed(self, height = 1.4):
        if not self.pos_got_ever:
            print("[WARNING] Uav position never got, will takeoff open-loop if you confirm again")
            global takeoff_confirm_without_position
            if takeoff_confirm_without_position:
                self.uav_take_off()
            takeoff_confirm_without_position = True
        else:
            height_before_takeoff = self.pose.z
            target_height = height_before_takeoff + height
            for i in range(200):
                # 最多两百次
                if not self.ctrl_block_flag:
                    print("[INFO] takeoff process stopped")
                    break
                if math.fabs(self.pose.z - self.arm_height) < target_height: # in case of NED
                    self.uav_send_speed_FLU(0,0,0.2)
                else:
                    print(f"[INFO] UAV get the target height at {self.pose.z}")
                    break
    
    def arm_uav(self):
        self.fcu_master.mav.command_long_send(
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,  # Confirmation
            1,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        self.arm_height = self.pose.z

    def arm_and_takeoff(self):
        self.arm_uav()
        time.sleep(0.1)
        self.arm_uav()
        time.sleep(0.1)
        self.uav_take_off()

    def takeoff_and_turn(self, angle_degree = 180):
        self.arm_uav()
        time.sleep(0.1)
        self.arm_uav()
        time.sleep(0.1)
        self.uav_take_off()
        t = angle_degree / 60.
        for i in range(int(10*t)):
            self.uav_turn_yaw_angle(i*6)
            if i % 3 == 0:
                print("turning: " + str(i*6) + " degree.")
            time.sleep(0.1)
        
    def dis_arm_uav(self):
        self.fcu_master.mav.command_long_send(
            self.fcu_master.target_system, self.fcu_master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            0,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        
    def data_gcs2fcu_t(self):
        while True:
            try:
                # 从地面站接收消息
                msg = self.gcs_master_udpin.recv_match(blocking=True)
                if not msg:
                    continue
                print(msg)
                # 检查消息类型
                if msg.get_type() in ['VISION_POSITION_ESTIMATE', 'SET_POSITION_TARGET_LOCAL_NED', 'HEARTBEAT']:
                    # 将消息原样转发到飞控
                    continue
                    self.fcu_master.mav.send(msg)
            except KeyboardInterrupt:
                print("程序被用户中断")
                break

    def start_data_fcu2computer(self):
        import threading
        t = threading.Thread(target=self.data_fcu2gcs_t)
        t.start

    def unpack_do_set_param(self, msg):
        msg : dict
        param_num   = msg['param1']
        
        params      = msg['param2']
        pass
    
    def send_target_pose_img(self):
        self.gcs_master_udpout.mav.command_long_send(
            255, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
            DrvCmd.DRV_INFO_TARGET_POS_IMG, self.detect_result.target_pos[0], self.detect_result.target_pos[1], self.detect_result.target_pos[2], self.detect_result.target_pos[3], 0, 0
        )
        
    def send_target_pose_est(self):
        self.gcs_master_udpout.mav.command_long_send(
            255, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
            DrvCmd.DRV_INFO_TARGET_POS_EST, self.detect_result.est_pos[0], self.detect_result.est_pos[1], 0, 0, 0, 0
        )
        
    def set_mode(self):
        # Set mode to OFFBOARD
        self.fcu_master.mav.command_long_send(
            self.fcu_master.target_system,
            self.fcu_master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Mode flag
            6,  # Custom mode: OFFBOARD mode
            0, 0, 0, 0, 0
        )
        
    def data_gcs2computer(self):
        while not self.should_shutdown:
            try:
                # 从地面站接收指令
                msg = self.gcs_master_udpin.recv_match(blocking=False)
                # print(msg)
                if not msg:
                    continue
                if msg.get_type() == 'COMMAND_LONG':
                    msg = msg.to_dict()
                    cmd = msg['command']
                    conf = msg['confirmation']
                    param1 = msg['param1']
                    param2 = msg['param2']
                    param3 = msg['param3']
                    param4 = msg['param4']
                    param5 = msg['param5']
                    param6 = msg['param6']
                    param7 = msg['param7']
                    if cmd != mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
                        # 限制控制线程
                        self.mission_state = DrvCmd.DRV_CMD_TASK_WAIT
                    if cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        takeoff_height = param1
                        print(f"[CMD] TAKOFF {takeoff_height}")
                        if takeoff_height == 0:
                            self.uav_take_off()
                        else:
                            pass
                        pass
                    elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        print(f"[CMD] LAND")
                        self.uav_land()
                        pass
                    elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        arm_flag = param1
                        print(f"[CMD] ARM" if arm_flag else "[CMD] DISARM")
                        if arm_flag:
                            self.arm_uav()
                        else:
                            self.dis_arm_uav()
                        pass
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
                        mission_current = param1
                        #TODO 0525
                        print(f"[CMD] MISSION CURRENT {mission_current}")
                        if mission_current >=0 and mission_current <= DrvCmd.DRV_CMD_EMERGENCY_STOP:
                            self.mission_state = mission_current
                            self.outer_enable = False
                        else:
                            self.outer_enable = True
                        if mission_current == DrvCmd.DRV_CMD_EMERGENCY_STOP:
                            self.uav_land()
                        if mission_current == DrvCmd.DRV_INFO_AGENT_POS_GLOBAL:
                            pass
                        elif mission_current == DrvCmd.DRV_INFO_AGENT_TASK_NOW:
                            pass
                        elif mission_current == DrvCmd.DRV_INFO_TARGET_POS_EST:
                            pass
                        elif mission_current == DrvCmd.DRV_INFO_TARGET_POS_GLOBAL:
                            pass
                        elif mission_current == DrvCmd.DRV_INFO_TARGET_POS_IMG:
                            pass
                        elif mission_current == DrvCmd.DRV_CMD_OFFBOARD:
                            print(f"[CMD] OFFBOARD CONTROL")
                            self.set_mode()
                            self.send_offboard_command()
                        elif mission_current == DrvCmd.DRV_CMD_SETGROUP:
                            self.group = param2
                            print(f'-------------{self.group}----------------')
                            print(f"[CMD] SET GROUP {self.group}")
                            print(f'-------------{self.group}----------------')
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE:
                        continue_flag = param1  # 1: continue, 0: stop
                        print(f"[CMD] CONTINUE" if continue_flag else f"[CMD] STOP")
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER:
                        self.unpack_do_set_param(msg)
                        pass
                elif msg.get_type() == 'VICON_POSITION_ESTIMATE':
                    msg = msg.to_dict()
                    self.pos_source = UAVOnBoard.POS_SRC_VICON
                    x = msg['x']
                    y = msg['y']
                    z = msg['z']
                    roll = msg['roll']
                    pitch = msg['pitch']
                    yaw = msg['yaw']
                    self.pose.x = x
                    self.pose.y = y
                    self.pose.z = z
                    self.pose.r = roll
                    self.pose.p = pitch
                    self.pose.yaw = yaw
                    self.pos_got_ever = True
                    pass
            except KeyboardInterrupt:
                break
            except TimeoutError:
                pass
            except socket.timeout:
                pass
            
    def data_fcu2gcs_t(self):
        last_print_time = time.time()
        print("begin to trans from fcu")
        while not self.should_shutdown:
            try:
                # 从飞控接收消息
                should_print = False
                msg = self.fcu_master.recv_match(blocking=False)
                if not msg:
                    continue
                # print(msg.get_type())
                msg_type = msg.get_type()
                if msg_type == 'COMMAND_ACK':
                    continue
                if msg_type == 'BAD_DATA':
                    continue
                if time.time() - last_print_time > 0.1:
                    #TODO
                    should_print = False
                    last_print_time = time.time()
                    # print(msg.get_type())
                if msg_type == 'HEARTBEAT':
                    self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    if should_print:
                        print('heart beat')
                if msg_type == 'LOCAL_POSITION_NED':
                    m = msg.to_dict()
                    if should_print:
                        print(f"飞机位置NED："+
                            f"x : {m['x'] :.2f} "+
                            f"y : {m['y'] :.2f} "+
                            f"z : {m['z'] :.2f} "+
                            f"vx: {m['vx']:.2f} "+
                            f"vy: {m['vy']:.2f} "+
                            f"vz: {m['vz']:.2f} "
                            )
                    if self.pos_source == UAVOnBoard.POS_SRC_NONE or self.pos_source == UAVOnBoard.POS_SRC_LOCAL:
                        # 尚未置位何种来源
                        # TODO 不要收到就发送
                        self.pos_source = UAVOnBoard.POS_SRC_LOCAL
                        self.pose.x = m['x']
                        self.pose.y = m['y']
                        self.pose.z = m['z']
                        self.vel.x = m['vx']
                        self.vel.y = m['vy']
                        self.vel.z = m['vz']
                        self.pos_got_ever = True
                        self.gcs_master_udpout.mav.local_position_ned_send(0, self.pose.x, self.pose.y, self.pose.z, self.vel.x, self.vel.y, self.vel.z)
                if msg_type == 'ATTITUDE':
                    m = msg.to_dict()
                    if should_print:
                        print(f"飞机姿态："+
                            f"roll : {m['roll']:.2f} "+
                            f"pitch: {m['pitch']:.2f} "+
                            f"yaw  : {m['yaw']:.2f} "
                            )
                    if self.pos_source == UAVOnBoard.POS_SRC_NONE or self.pos_source == UAVOnBoard.POS_SRC_LOCAL:
                        self.pose.r = m['roll']
                        self.pose.p = m['pitch']
                        self.pose.yaw = m['yaw']
                        self.gcs_master_udpout.mav.attitude_send(0, self.pose.r, self.pose.p, self.pose.yaw, 0,0,0)
                    
                if msg_type == 'GLOBAL_POSITION_INT':
                    if should_print:
                        print(msg.to_dict())
                # print(msg.get_type())
                # 检查消息类型
                elif msg_type == 'BATTERY_STATUS':
                    m = msg.to_dict()
                    self.battery = m['voltages'][0]
                elif msg_type == 'DISTANCE_SENSOR':
                    m = msg.to_dict()
                    self.lidar_height = m['current_distance']
                elif msg_type == 'HIGHRES_IMU':
                    m = msg.to_dict()
                    self.imu_acc = [m['xacc'], m['yacc'], m['zacc']]
                    self.imu_gyro = [m['xgyro'], m['ygyro'], m['zgyro']]    
                elif msg_type == 'ODOMETRY':
                    m = msg.to_dict()
                    self.odom_xyz = [m['x'], m['y'], m['z']]
                    self.odom_q   = [m['q'][0], m['q'][1], m['q'][2], m['q'][3]]
                    self.odom_v   = [m['vx'], m['vy'], m['vz']]
                elif msg_type == 'SERVO_OUTPUT_RAW':
                    m = msg.to_dict()
                    self.servos = [m['servo1_raw'], m['servo2_raw'], m['servo3_raw'], m['servo4_raw']]
                elif msg_type == 'TIME_SYNC':
                    m = msg.to_dict()
                    self.time_sync = m['ts1']
                elif msg_type == 'VFR_HUD':
                    m = msg.to_dict()
                    self.vfr_hud = [m['airspeed'], m['groundspeed'], m['heading'], m['throttle'], m['alt'], m['climb']]
                elif msg_type == 'OPTICAL_FLOW_RAD':
                    m = msg.to_dict()
                    self.opti_xyq = [m['integrated_x'], m['integrated_y'], m['quality']]
                    # print(self.opti_xyq)
                elif msg_type == 'GPS_RAW_INT':
                    m = msg.to_dict()
                    self.gps = [float(msg.lat) / 1.e7, float(msg.lon) / 1.e7]
                if msg_type not in ['BAD_DATA']:  # 忽略无效数据 TODO 需要检测
                    # 将消息原样转发到地面站
                    continue
                    self.gcs_master_udpout.mav.send(msg)
            except KeyboardInterrupt:
                print("程序被用户中断")
                break

def test_ctrl_t():
    """测试线程
    """
    # Uav.start_send_vel_cmd_t()
    input("arm?")
    Uav.arm_uav()
    time.sleep(0.05)
    Uav.arm_uav()
    time.sleep(0.05)
    Uav.arm_uav()
    time.sleep(0.05)
    input("take off?")
    # Uav.uav_take_off()
    Uav.uav_takeoff_opened()
    # Uav.uav_send_pose_ned(0,0,-0.8)
    input("go?")
    
    # while not math.fabs(Uav.pose.z) > 0.8:
    #     print(f"taking off: z: {Uav.pose.z}")
    #     time.sleep(0.2)
        
    for i in range(50):
        # Uav.uav_hover()
        print("悬停")
        time.sleep(0.1)
    for i in range(50):
        # Uav.uav_send_speed_FLU(0.3, 0.0, 0.0)
        Uav.vel_set_frd.x = 0.3
        Uav.vel_set_frd.y = 0.0
        Uav.vel_set_frd.z = 0.0
        print("向前飞行")
        time.sleep(0.1)
        
    for i in range(50):
        # Uav.uav_send_speed_FLU(0.0, -0.3, 0.0)
        Uav.vel_set_frd.x = 0.0
        Uav.vel_set_frd.y = 0.3
        Uav.vel_set_frd.z = 0.0
        print("向右飞行")
        time.sleep(0.1)
    for i in range(50):
        Uav.vel_set_frd.x = 0.0
        Uav.vel_set_frd.y = -0.3
        Uav.vel_set_frd.z = 0.0
        print("向左飞行")
        time.sleep(0.1)
    Uav.vel_set_frd.x = 0.0
    Uav.vel_set_frd.y = 0.0
    Uav.vel_set_frd.z = 0.0
    print("悬停")
    time.sleep(2)
    print("降落")
    Uav.uav_land_opened()
    # Uav.uav_land()
    # Uav.dis_arm_uav()

def test_turn():
    angle = 50
    print(f"turn to {angle}")
    Uav.uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    Uav.uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    Uav.uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    Uav.uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    Uav.uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    Uav.uav_land()

if __name__ == "__main__":
    # test_ctrl_t()
    # test_turn()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--uav_id", type=int, default=0)
    parser.add_argument("--test", type=int, default=0)
    parser.add_argument("--ros_on", type=int, default=0)
    args = parser.parse_args()
    import os
    UAV_ID = os.environ.get("UAV_ID")
    Uav = UAVOnBoard(ros_on=args.ros_on)
    if args.test == 1:
        Uav.start_send_vel_cmd_t()
        
        test_ctrl_t()
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("程序被用户中断")
            Uav.should_shutdown = True
            break
    # t1 = threading.Thread(target = Uav.data_gcs2computer)
    # t2 = threading.Thread(target = Uav.data_fcu2gcs_t)
    # t1.start()
    # t2.start()