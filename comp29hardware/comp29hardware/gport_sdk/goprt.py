import serial
import struct
from .crc32 import CRC32Software
import threading

class rpy:
    def __init__(self, r=0, p=0, y=0) -> None:
        self.r = r
        self.p = p
        self.y = y
        
class GportGimbal:
    GPORT_FRAME_HEADER          = 0xAE
    GPORT_FRAME_VERSION         = 0x01
    
    GPORT_FRAME_ORDER_CTRL      = 0x85
    GPROT_FRAME_GIMBAL_STATE    = 0x87
    
    GPORT_CTRL_MODE_SPEED       = 0x01
    GPORT_CTRL_MODE_ANGLE       = 0x02
    GPORT_CTRL_MODE_CENTER      = 0x03
    
    # GPORT_MAX_PITCH             = 45
    # GPORT_MIN_PITCH             = -90
    
    # GPORT_MAX_YAW               = 90
    # GPORT_MIN_YAW               = -90
    
    # GPORT_MAX_ROLL              = 45
    # GPORT_MIN_ROLL              = -45
    def __init__(self, port, baudrate=115200) -> None:
        self.print_tip()
        self.imu_rpy = rpy()
        self.hall_rpy = rpy()
        self.ser = serial.Serial(port, baudrate)
        self.ser.timeout = 0.5
        self.should_stop = False
        self.listen_thread = threading.Thread(target=self.listen_t)
        self.listen_thread.start()
        pass
    
    def print_tip(self):
        print("GportGimbal")
        print("相机正置坐标系为FLU，倒置坐标系为FRD")
        print("相机倒置为例：")
        print("向前yaw为零，  向右yaw为正，  向左yaw为负")
        print("水平pitch为零，向上pitch为正，向下pitch为负")
        print("使用 hall_rpy 属性获取当前相机的角度")
        
    def __del__(self):
        self.stop()
    
    def get_hall_rpy(self):
        return self.hall_rpy
    
    def stop(self):
        self.should_stop = True
        self.listen_thread.join()
        self.ser.close()
        
    def listen_t(self):
        while not self.should_stop:
            try:
                header = self.ser.read(1)
                if header[0] == self.GPORT_FRAME_HEADER:
                    ver = self.ser.read(1)
                    if ver[0] != 0x01: 
                        continue
                    length = self.ser.read(1)
                    order, header_sum = struct.unpack("<BB", self.ser.read(2))
                    msg = self.ser.read(int(length[0]))
                    crc = self.ser.read(4)
                    if order == self.GPROT_FRAME_GIMBAL_STATE:
                        msg_crc = CRC32Software(msg)
                        if msg_crc != struct.unpack("<I", crc):
                            pass
                            # print("crc error")
                            # continue
                        imu_r, imu_p, imu_y, hall_r, hall_p, hall_y = struct.unpack("<6h", msg)
                        self.imu_rpy = rpy(imu_r/100, imu_p/100, imu_y/100)
                        self.hall_rpy = rpy(hall_r/100, hall_p/100, hall_y/100)
                        # print(self.imu_rpy.r, self.imu_rpy.p, self.imu_rpy.y)
                        # print(self.hall_rp.r, self.hall_rp.p, self.hall_rp.y)
            except serial.SerialTimeoutException:
                pass
            except IndexError:
                pass
            except Exception as e:
                print(e)
                pass
    def send_msg(self, order_num, msg):
        length = len(msg)
        header = struct.pack("<BBBB", self.GPORT_FRAME_HEADER, self.GPORT_FRAME_VERSION, length, order_num)
        header_sum = self.GPORT_FRAME_VERSION + length + order_num
        header_sum = header_sum & 0xFF
        msg_crc = CRC32Software(msg)
        msg = header + struct.pack("<B", header_sum) + msg + struct.pack("<I", msg_crc)
        self.ser.write(msg)
    
    def send_ctrl_msg(self, mode, roll, pitch, yaw, v_roll, v_pitch, v_yaw):
        payload = struct.pack("<B6h",
                                mode, 
                                int(roll*100),
                                int(pitch*100), 
                                int(yaw*100),
                                int(v_roll*100),
                                int(v_pitch*100),
                                int(v_yaw*100))
        self.send_msg(self.GPORT_FRAME_ORDER_CTRL, payload)
        
        
    def set_angle_degree(self, roll, pitch, yaw):
        self.send_ctrl_msg(self.GPORT_CTRL_MODE_ANGLE, roll, pitch, yaw, 0, 0, 0)
        pass
    
    def set_speed_degree(self, v_roll, v_pitch, v_yaw):
        self.send_ctrl_msg(self.GPORT_CTRL_MODE_SPEED, 0,0,0, v_roll, v_pitch, v_yaw)
        
    def set_central(self):
        self.send_ctrl_msg(self.GPORT_CTRL_MODE_CENTER, 0,0,0,0,0,0)
    
    def print_rpy(self):
        print()
        print(f"imu:  roll: {self.imu_rpy.r:6.2f}, pitch: {self.imu_rpy.p:6.2f}, yaw: {self.imu_rpy.y:6.2f}")
        print(f"hall: roll: {self.hall_rpy.r:6.2f}, pitch: {self.hall_rpy.p:6.2f}, yaw: {self.hall_rpy.y:6.2f}")
if __name__ == "__main__":
    gimbal = GportGimbal("COM8")
    import time
    gimbal.set_angle_degree(0,0,0)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.set_angle_degree(0,10,0)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.set_angle_degree(0,-20,0)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.set_angle_degree(0,0,30)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.set_angle_degree(0,0,-30)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.set_angle_degree(0,0,0)
    time.sleep(1)
    gimbal.print_rpy()
    input("press enter to continue")
    
    gimbal.should_stop = True
    pass