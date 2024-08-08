import json
import logging
import os
import socket
import warnings
import serial
import struct
from functools import reduce
from config import DrvCmd, DetectResult
from pymavlink import mavutil
import threading

UART_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
FRAME_HEADER = b'\x7B'
FRAME_END = b'\x7D'
RECEIVE_DATA_SIZE = 24
SEND_DATA_SIZE    = 11 
SEND_FORMATE_NO_END      = '>cBBhBBh'
RECV_FORMATE             = '>cB10hBB'

class Vec3:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        pass
    
    def set_vec(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        pass
    
class IMUData:
    def __init__(self) -> None:
        self.acc = Vec3()
        self.ang = Vec3()
        pass
    
    def set_acc(self, x, y, z):
        self.acc.set_vec(x, y, z)
        pass
    
    def set_ang(self, x, y, z):
        self.ang.set_vec(x, y, z)
        pass
def odom_trans(x):
    x = x/1000 + (x %1000) * 0.001
    return x

ACCEl_RATIO     = 1671.84 
GYROSCOPE_RATIO = 0.00026644

def acc_trans(x):
    x = x/ACCEl_RATIO
    return x
    
def geo_trans(x):
    x = x*GYROSCOPE_RATIO
    return x

class Pose:
    def __init__(self, x=0, y=0, z=0, r=0, p=0, yaw=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.p = p
        self.yaw = yaw
        pass

class CarCfg:
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
        else: Warning("Config file type not supported")
        
    def get_config(self):
        return self.config
    
    def load_config(self, file_name):
        with open(file_name, "r") as f:
            config = json.load(f)
        return config
    
class WheeltecCar:
    POS_SRC_NONE  = -1
    POS_SRC_VICON = 0
    POS_SRC_LOCAL = 1
    POS_SRC_GPS   = 2
    POS_SRC_CALCULATED = 3
    
    def __init__(self, config_file = "car_config.json") -> None:
        self.pos_source = self.POS_SRC_NONE # Position source
        self.mission_state = DrvCmd.DRV_CMD_TASK_WAIT
        self.pose_goto = [0,0,0]
        self.detect_result = DetectResult()
        self.pos_got_ever = False
        self.takeoff_confirm_without_position = False
        self.ctrl_block_flag = True  # 用于其他任务暂停起飞和降落的标志, True: 可以持续运行，False: 立刻截止当前任务
        self.video_ip = None
        if config_file is None:
            pass
        else:
            if isinstance(config_file, str):
                if not os.path.exists(config_file):
                    warnings.warn("配置文件不存在，请检查配置文件路径\nConfig file not found, please check the path")
                self.car_cfg = CarCfg(config_file).get_config()
            if isinstance(config_file, dict):
                self.car_cfg = config_file
            self.gcs_protocol = self.car_cfg['CONNECTION']['GCS_CONNECTION']["GCS_PROTOCOL"]
            print(self.gcs_protocol)
            self.gcs_config = self.car_cfg['CONNECTION']['GCS_CONNECTION']
            self.fcu_config = self.car_cfg['CONNECTION']['FCU_CONNECTION']
            self.video_on = self.gcs_config["VIDEO_ON"]
            if self.video_on:
                self.video_protocol = self.gcs_config["VIDEO_PROTOCOL"]
                self.video_ip = self.gcs_config["VIDEO_IP"]
                self.video_port = self.gcs_config["VIDEO_PORT"]
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
        self.video_ip = self.gcs_ip if self.video_ip is None else self.video_ip
        self.open_flag = True
        self.outer_enable = True
        self.pose = Pose()
        self.vel = Vec3()
        self.imu = IMUData()
        self.should_stop = False
        self.fcu_ser = None
        self.gcs_master_udpin = None
        self.gcs_master_udpout = None
        self.onboard_connection_init()
        t1 = threading.Thread(target=self.uart_listen_t)
        t2 = threading.Thread(target=self.data_gcs2computer)
        t1.start()
        t2.start()
        pass
    
    def onboard_connection_init(self):
        try:
            self.fcu_ser = serial.Serial(self.fcu_port, self.fcu_baud)
            print("车控连接成功@" + self.fcu_port + " " + str(self.fcu_baud))
            print("Car FCU connected@" + self.fcu_port + " " + str(self.fcu_baud))
        except Exception as e:
            print("车控连接失败，请检查串口顺序，并使能串口\n Error in connecting to fcu, please check the port and baudrate")
            print("车控连接配置如下：\n" + f"Port: {self.fcu_port}\nBaudrate: {self.fcu_baud}")
            print(e)
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
                self.gcs_master_udpin = mavutil.mavlink_connection(f"udpin:{self.local_ip}:{self.msg_in_port_local}",source_system=255, source_component=1, udp_timeout=1, timeout=1)
                self.gcs_master_udpin.mav.version = 'v2.0'
                print("打开udpin网口成功@" + f"udpin:{self.local_ip}:{self.msg_in_port_local}")
            except Exception as e:
                print(e)
                self.open_flag = False
                warnings.warn("打开udpin网口出现错误，无法连接地面站，请检查串口顺序，并使能串口\n Error in connecting to fcu or gcs, please check the port and baudrate") 
    
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
        
    def __send_vel(self, vx, wz):
        vx *= 1000
        wz *= 1000
        data = struct.pack(SEND_FORMATE_NO_END, FRAME_HEADER, 0, 0, int(vx), 0, 0, int(wz))
        chk_sum = self.check_sum(data)
        data += struct.pack('<Bc', chk_sum, FRAME_END)
        self.fcu_ser.write(data)
        
    def send_vel(self, vx, wz):
        # print(11111)
        if self.outer_enable:
            # print("cccccc")
            self.__send_vel(vx, wz)
    
    def check_sum(self, data):
        return reduce(lambda x, y: x^y, data)
        chk_sum = 0
        for i in range(len):
            chk_sum ^= data[i]
        return chk_sum

    def uart_listen_t(self):
        self.pt = 0
        if self.fcu_ser is None:
            print("!!!"*50)
            print("FCU not connected")
            return
        while not self.should_stop:
            self.pt += 1
            data = self.fcu_ser.read(1)
            # print(data, data==FRAME_HEADER)
            if data != FRAME_HEADER:
                continue
            data += self.fcu_ser.read(RECEIVE_DATA_SIZE - 1)
            # print(data[-1], data[-1]==ord(FRAME_END))
            
            if data[-1] != ord(FRAME_END):
                continue
            chk_sum = self.check_sum(data[:-2])
            if chk_sum != data[-2]:
                print("Check sum error")
                continue
            data_recv = struct.unpack(RECV_FORMATE, data)
            self.vel.set_vec( odom_trans(data_recv[2]), odom_trans(data_recv[3]), odom_trans(data_recv[4]))
            self.imu.set_acc( acc_trans(data_recv[5]), acc_trans(data_recv[6]), acc_trans(data_recv[7]))
            self.imu.set_ang( geo_trans(data_recv[8]), geo_trans(data_recv[9]), geo_trans(data_recv[10]))
            if self.pt % 3 == 0:
                self.gcs_master_udpout.mav.attitude_send(0, 0,0,0,self.imu.ang.x, self.imu.ang.y, self.imu.ang.z)
                # self.gcs_master_udpout.mav.attitude_send(0, self.imu.ang.x, self.imu.ang.y, self.imu.ang.z, 0,0,0)
                self.gcs_master_udpout.mav.local_position_ned_send(0, self.pose.x, self.pose.y, self.pose.z, self.vel.x, self.vel.y, self.vel.z)
                
                # self.print_data()
    
    
    def print_data(self):
        print("==================================================================")
        print(f"Vel: {self.vel.x:7.2f}, {self.vel.y:7.2f}, {self.vel.z:7.2f}")
        print(f"Acc: {self.imu.acc.x:7.2f}, { self.imu.acc.y:7.2f}, {self.imu.acc.z:7.2f}")
        print(f"Ang: {self.imu.ang.x:7.2f}, { self.imu.ang.y:7.2f}, { self.imu.ang.z:7.2f}")
        print()
        
    def stop(self):
        self.should_stop = True
        self.send_vel(0, 0)
        pass
    
    def unpack_do_set_param(self, msg):
        msg : dict
        param_num   = msg['param1']
        
        params      = msg['param2']
        pass
    
    
    def data_gcs2computer(self):
        if self.gcs_master_udpin is None:
            print("GCS not connected")
            return
        while not self.should_stop:
            try:
                # 从地面站接收指令
                # print("ready to receive")
                msg = self.gcs_master_udpin.recv_match(blocking=True, timeout=1)
                # msg = self.gcs_master_udpin.recv_msg()
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
                            print("NO TAKE OFF FOR CAR")
                            # self.uav_take_off()
                        else:
                            pass
                        pass
                    elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        print(f"[CMD] LAND")
                        print("NO LAND FOR CAR")
                        # self.uav_land()
                        pass
                    elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        arm_flag = param1
                        print(f"[CMD] ARM" if arm_flag else "[CMD] DISARM")
                        print("NO ARM FOR CAR")
                        # if arm_flag:
                        #     self.arm_uav()
                        # else:
                        #     self.dis_arm_uav()
                        # pass
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
                        mission_current = param1
                        print(f"[CMD] MISSION CURRENT {mission_current}")
                        if mission_current >=0 and mission_current <= DrvCmd.DRV_CMD_EMERGENCY_STOP:
                            self.mission_state = mission_current
                            self.outer_enable = True
                        if mission_current == DrvCmd.DRV_CMD_EMERGENCY_STOP:
                            self.outer_enable = False
                        if mission_current == DrvCmd.DRV_CMD_EMERGENCY_STOP:
                            self.send_vel(0,0)
                        if mission_current == DrvCmd.DRV_CMD_TASK_GOTO_WAIT:
                            self.pose_goto = [param2, param3, param4]
                            print(f"[CMD] GOTO {self.pose_goto}")
                            pass
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
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE:
                        continue_flag = param1  # 1: continue, 0: stop
                        print(f"[CMD] CONTINUE" if continue_flag else f"[CMD] STOP")
                    elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER:
                        self.unpack_do_set_param(msg)
                        pass
                elif msg.get_type() == 'VICON_POSITION_ESTIMATE':
                    msg = msg.to_dict()
                    self.pos_source = WheeltecCar.POS_SRC_VICON
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
                print("time out")
                pass
            except socket.timeout:
                pass

if __name__ == "__main__":
    import time
    car = WheeltecCar()
    # car.onboard_connection_init()
    # car.send_vel(0.2, 0)
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            car.stop()
            print("stop")
            break
    
    # car.send_vel(0, 0.2)
    # time.sleep(2)
    # car.send_vel(0, 0)