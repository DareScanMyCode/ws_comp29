# from .gport_sdk.goprt import GportGimbal
from .GcsOnboardClient.python.UavOnboard import UAVOnBoard
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from quaternions import Quaternion as Quaternion
import math
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from comp29hardware.GcsOnboardClient.python.config import DrvCmd
from sensor_msgs.msg        import Imu
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import Float32MultiArray
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped
from std_msgs.msg           import Int64, Float64, Bool

class Uav2ROSNode(Node):
    def __init__(self, args):
        super().__init__('uwb2ros_node')
        print(args)
        
        # 获取参数
        self.uav_id             = int(os.getenv('UAV_ID', '-1'))
        if self.uav_id != -1:
            self.get_logger().info(f"使用环境变量作为UAV ID: {self.uav_id}")
        else:
            self.get_logger().info(f"尝试使用ROS参数作为UAV ID:")
            self.uav_id             = self.declare_parameter('uav_id', -1).get_parameter_value().integer_value
        if self.uav_id != -1:
            default_cfg_file = f"/home/cat/ws_comp29/src/configs/UAV_configs/config{self.uav_id}.json"
            self.get_logger().info(f"使用环境变量指定config file: {self.uav_id}")
        else:
            default_cfg_file = f"/home/cat/ws_comp29/configs/UAV_configs/config1.json"
            
        self.config_file        = self.declare_parameter('uav_config_file',default_cfg_file).get_parameter_value().string_value
        # TODO 是否修改成同步发送，收到信息就发送？需要修改UAVOnboard程序
        self.fcu_pub_fps        = self.declare_parameter('fcu_pub_fps', 20.0).get_parameter_value().double_value
        
        self.mis_state_pub      = self.create_publisher(Int64,             'super_mission_state', 1)
        self.imu_pub            = self.create_publisher(Imu,               'imu', 1)
        self.servo_pub          = self.create_publisher(Float32MultiArray, 'servo_data', 1)
        self.attitude_tgt_pub   = self.create_publisher(Float32MultiArray, 'attitude_setpt', 1)
        self.local_pos_pub      = self.create_publisher(PoseStamped,       'local_position_ned', 1)
        self.gps_pos_pub        = self.create_publisher(PoseStamped,       'gps_position', 1)
        self.velocity_pub       = self.create_publisher(TwistStamped,      'velocity_and_angular', 1)
        self.lidar_height_pub   = self.create_publisher(Float64,           'lidar_height', 1)
        self.arm_state_pub      = self.create_publisher(Bool,              'arm_state', 1)
        
        self.last_vel_cmd_got_time = None
        
        # 初始化FCU
        try:
            self.uav = UAVOnBoard(self.config_file)
            self.uav.set_mode()
            self.uav.send_offboard_command()
            self.uav.start_send_vel_cmd_t()
            self.get_logger().info(f"FCU配置完成")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize gimbal: {e}")
            return
        # 打印参数
        self.get_logger().info(f'Using config file: {self.config_file}')
        self.get_logger().info(f'pub fps: {self.fcu_pub_fps}')
        # 创建QoS配置文件
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                     # 历史消息的队列长度
        )
        
        # 创建订阅者并应用QoS配置
        self.vel_frd_sub        = self.create_subscription(
            TwistStamped, 
            '/vel_set_frd', 
            self.vel_set_frd_cb, 
            qos_profile
        )
        
        self.vel_ned_sub        = self.create_subscription(
            TwistStamped, 
            '/vel_set_ned', 
            self.vel_set_ned_cb, 
            qos_profile
        )
        
        # 创建订阅者，订阅/arm话题
        self.arm_flag_sub = self.create_subscription(
            Bool,
            '/arm',
            self.arm_cb,
            10  # QoS 配置
        )
        
        self.arm_flag = False  # 标志变量，跟踪是否需要进行arm操作
        
        self.vel_frd_sub
        self.vel_ned_sub
        self.signal_lost = False  # 标志变量，跟踪信号丢失状态
        self.cmd_watch_timer = self.create_timer(0.4, self.vel_watch)
        self.pub_timer = self.create_timer(1.0/self.fcu_pub_fps, self.pub_t)
    
    def run(self):
        while rclpy.ok():
            # self.get_logger().info("==============================")
            rclpy.spin_once(self, timeout_sec=0.3)
            # time.sleep(0.05)
        self.get_logger().info("END")
        
    
    def arm_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("[UAV] 收到arm信号，开始进行arm操作")
            for i in range(100):
                self.uav.arm_uav()
                if self.uav.is_armed:
                    self.get_logger().info("[UAV] 无人机已经解锁")
                    break
                time.sleep(0.04)
            self.get_logger().info("[UAV] 无人机解锁失败！！！！！！！！！！！！！！！")
            
    
    def takeoff_cb(self, msg):
        pass
    
    def land_cb(self, msg):
        pass
    
    def vel_watch(self):
        current_time = time.time()
        if self.last_vel_cmd_got_time is not None:
            if current_time - self.last_vel_cmd_got_time > 1.0:
                if not self.signal_lost:
                    self.get_logger().info("长时间未收到消息，设置速度为零")
                    self.uav.vel_set_ned.x = 0.0
                    self.uav.vel_set_ned.y = 0.0
                    self.uav.vel_set_ned.z = 0.0
                    
                    self.uav.vel_set_frd.x = 0.0
                    self.uav.vel_set_frd.y = 0.0
                    self.uav.vel_set_frd.z = 0.0
                    self.signal_lost = True  # 设置信号丢失标志
            else:
                if self.signal_lost:
                    self.get_logger().info("速度信号恢复")
                    self.signal_lost = False  # 重置信号丢失标志
                
    def vel_set_frd_cb(self, msg: TwistStamped):
        self.last_vel_cmd_got_time = time.time()
        # self.get_logger().info("got frd vel msg")
        # self.uav.uav_send_speed_FRD(msg.twist.linear.x,
        #                             msg.twist.linear.y,
        #                             msg.twist.linear.z)
        self.uav.vel_set_frd.x = msg.twist.linear.x
        self.uav.vel_set_frd.y = msg.twist.linear.y
        self.uav.vel_set_frd.z = msg.twist.linear.z
        self.uav.vel_mode      = self.uav.VEL_MODE_FRD
        pass
    
    def vel_set_ned_cb(self, msg: TwistStamped):
        self.last_vel_cmd_got_time = time.time()
        # self.get_logger().info("got ned vel msg")
        # self.uav.uav_send_speed_ned(msg.twist.linear.x,
        #                             msg.twist.linear.y,
        #                             msg.twist.linear.z)
        self.uav.vel_set_ned.x = msg.twist.linear.x
        self.uav.vel_set_ned.y = msg.twist.linear.y
        self.uav.vel_set_ned.z = msg.twist.linear.z
        self.uav.vel_mode      = self.uav.VEL_MODE_NED
        pass
    
    def pub_t(self):
        # rate = self.create_rate(float(self.fcu_pub_fps))
        # TODO 等待所有消息都收到了？
        # rclpy.spin_once(self)
        should_pub_imu              = True
        should_pub_servo            = True
        should_pub_attitude_tgt     = True
        should_pub_local_pos        = True
        should_pub_gps              = True
        should_pub_vel              = True
        should_pub_lidar_height     = True
        mis_msg = Int64()
        if self.uav.mission_state == DrvCmd.DRV_CMD_TASK_WAIT:
            mis_msg.data = 0
        elif self.uav.mission_state == DrvCmd.DRV_CMD_TASK_READY:
            mis_msg.data = 1
        elif self.uav.mission_state == DrvCmd.DRV_CMD_TASK_BEGIN:
            mis_msg.data = 2
        elif self.uav.mission_state == DrvCmd.DRV_CMD_TASK_END:
            mis_msg.data = 3
        elif self.uav.mission_state == DrvCmd.DRV_CMD_EMERGENCY_STOP:
            mis_msg.data = 4
        self.mis_state_pub.publish(mis_msg)
        # pub IMU
        if should_pub_imu and self.uav.imu_acc is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = self.uav.imu_acc[0]
            imu_msg.linear_acceleration.y = self.uav.imu_acc[1]
            imu_msg.linear_acceleration.z = self.uav.imu_acc[2]
            imu_msg.angular_velocity.x = self.uav.imu_gyro[0]
            imu_msg.angular_velocity.y = self.uav.imu_gyro[1]
            imu_msg.angular_velocity.z = self.uav.imu_gyro[2]
            fcu_rpy = [ self.uav.pose.r, self.uav.pose.p, self.uav.pose.yaw ] # ROLL PITCH HEADING
            fcu_q   = Quaternion.from_euler(fcu_rpy, axes = ['z', 'y', 'x'])
            imu_msg.orientation.w = fcu_q.w
            imu_msg.orientation.x = fcu_q.x
            imu_msg.orientation.y = fcu_q.y
            imu_msg.orientation.z = fcu_q.z
            # 发布IMU数据
            self.imu_pub.publish(imu_msg)
        
        # pub servo
        if should_pub_servo and self.uav.servos is not None:
            servo_msg = Float32MultiArray()
            servo_msg.data = [float(self.uav.servos[i]) for i in range(4)]
            self.servo_pub.publish(servo_msg)
            
        # pub attitude-tgt
        
        # pub local-pos
        if should_pub_local_pos : #and self.uav.odom_q is not None:
            local_pos_msg = PoseStamped()
            local_pos_msg.header.stamp = self.get_clock().now().to_msg()
            local_pos_msg.header.frame_id = 'LOCAL_NED'
            
            local_pos_msg.pose.position.x = float(self.uav.pose.x)
            local_pos_msg.pose.position.y = float(self.uav.pose.y)
            local_pos_msg.pose.position.z = float(self.uav.pose.z)
            
            # local_pos_msg.pose.orientation.w = self.uav.odom_q[0]
            # local_pos_msg.pose.orientation.x = self.uav.odom_q[1]
            # local_pos_msg.pose.orientation.y = self.uav.odom_q[2]
            # local_pos_msg.pose.orientation.z = self.uav.odom_q[3]
            
            self.local_pos_pub.publish(local_pos_msg)
        
        # pub gps-pos
        if should_pub_gps and self.uav.gps is not None:
            gps_msg = PoseStamped()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'GPS'
            # x:lat, y:lon
            gps_msg.pose.position.x = self.uav.gps[0]
            gps_msg.pose.position.y = self.uav.gps[1]
            self.gps_pos_pub.publish(gps_msg)
        
        # pub vel
        if should_pub_vel and self.uav.imu_gyro is not None:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = 'LOCAL_NED'
            # linear
            vel_msg.twist.linear.x = self.uav.vel.x
            vel_msg.twist.linear.y = self.uav.vel.y
            vel_msg.twist.linear.z = self.uav.vel.z
            
            # angular
            vel_msg.twist.angular.x = self.uav.imu_gyro[0]
            vel_msg.twist.angular.y = self.uav.imu_gyro[1]
            vel_msg.twist.angular.z = self.uav.imu_gyro[2]
            self.velocity_pub.publish(vel_msg)
        
        
        # pub lidar-height
        if should_pub_lidar_height and self.uav.lidar_height is not None:
            lidar_hgt_msg = Float64()
            lidar_hgt_msg.data = self.uav.lidar_height / 100.0
            self.lidar_height_pub.publish(lidar_hgt_msg)
        # time.sleep(1.0/self.fcu_pub_fps)
        # rate.sleep()
        # print("UAV 结束1")

def main(args=None):
    rclpy.init()
    
    uav2ros_node = Uav2ROSNode(args)
    
    try:
        uav2ros_node.run()
    finally:
        print("UAV 线程结束")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
