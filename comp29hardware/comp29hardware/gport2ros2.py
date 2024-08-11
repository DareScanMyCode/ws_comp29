from .gport_sdk.goprt import GportGimbal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from quaternions import Quaternion as Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
import math
class Gimbal2ROSNode(Node):
    def __init__(self, args):
        super().__init__('uwb2ros_node')
        print(args)
        # 获取参数
        self.gimbal_port = self.declare_parameter('gimbal_port', '/dev/ttyS7').get_parameter_value().string_value
        self.gimbal_pub_fps = self.declare_parameter('gimbal_pub_fps', 5).get_parameter_value().integer_value
        self.gimbal_rpy_pub = self.create_publisher(PoseStamped, 'gimbalrpy_data', 10)
        self.gimbal_imu_rpy_pub = self.create_publisher(PoseStamped, 'gimbalimu_data', 10)

        # 初始化云台
        try:
            self.gimbal = GportGimbal(self.gimbal_port)
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize gimbal: {e}")
            return
        # 订阅
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            # history=QoSHistoryPolicy.SYSTEM_DEFAULT,         # 只保留最新的历史消息
            durability=DurabilityPolicy.VOLATILE,

            depth=1                                    # 历史消息的队列长度
        )
        
        self.gimbal_state_sub = self.create_subscription(PoseStamped, '/gimbalrpy_setpoint', self.gimbal_state_callback, qos_profile)
        
        # 打印参数
        self.get_logger().info(f'Using port: {self.gimbal_port}')
        self.get_logger().info(f'Publishing   gimbal rpy       data to: gimbalrpy_data')
        self.get_logger().info(f'Publishing   gimbal imu rpy   data to: gimbalimu_data')
    
    def gimbal_state_callback(self, msg: PoseStamped):
        r = msg.pose.position.x
        p = msg.pose.position.y
        y = msg.pose.position.z
        
        self.gimbal.set_angle_degree(r, p, y)
        self.get_logger().info(f'GIMBAL SET @ ROLL {r}, PITCH {p}, YAW {y}')
        
    def listing_t(self):
        rate = self.create_rate(int(self.gimbal_pub_fps))
        self.get_logger().info(f"gimbal thread begin @ {self.gimbal_pub_fps} FPS")
        while rclpy.ok() and not self.gimbal.should_stop:
            # 获取云台的当前姿态
            rpy = self.gimbal.get_hall_rpy()

            # 将欧拉角转换为四元数
            
            gimbal_q = Quaternion.from_euler([math.radians(rpy.r), math.radians(rpy.p), math.radians(rpy.y)], axes = ['z', 'y', 'x'])
            quaternion_msg = PoseStamped()
            quaternion_msg.header.stamp = self.get_clock().now().to_msg()
            quaternion_msg.header.frame_id = "gimbal"
            quaternion_msg.pose.position.x = float(rpy.r)
            quaternion_msg.pose.position.y = float(rpy.p)
            quaternion_msg.pose.position.z = float(rpy.y)
            quaternion_msg.pose.orientation.w = float(gimbal_q.w)
            quaternion_msg.pose.orientation.x = float(gimbal_q.x)
            quaternion_msg.pose.orientation.y = float(gimbal_q.y)
            quaternion_msg.pose.orientation.z = float(gimbal_q.z)

            # 发布四元数消息
            self.gimbal_rpy_pub.publish(quaternion_msg)


            rpy = self.gimbal.imu_rpy

            # 将欧拉角转换为四元数
            
            gimbal_q_imu = Quaternion.from_euler([math.radians(rpy.r), math.radians(rpy.p), math.radians(rpy.y)], axes = ['z', 'y', 'x'])
            imu_quaternion_msg = PoseStamped()
            imu_quaternion_msg.header.stamp = self.get_clock().now().to_msg()
            imu_quaternion_msg.pose.position.x = float(rpy.r)
            imu_quaternion_msg.pose.position.y = float(rpy.p)
            imu_quaternion_msg.pose.position.z = float(rpy.y)
            imu_quaternion_msg.pose.orientation.w = float(gimbal_q_imu.w)
            imu_quaternion_msg.pose.orientation.x = float(gimbal_q_imu.x)
            imu_quaternion_msg.pose.orientation.y = float(gimbal_q_imu.y)
            imu_quaternion_msg.pose.orientation.z = float(gimbal_q_imu.z)

            # 发布四元数消息
            self.gimbal_imu_rpy_pub.publish(imu_quaternion_msg)
            rate.sleep()
        
        self.gimbal.stop()
        
    def run(self):
        from threading import Thread
        t = Thread(target=self.listing_t)
        t.start()
        rclpy.spin(self)
        

def main(args=None):
    rclpy.init()
    
    gimbal2ros_node = Gimbal2ROSNode(args)
    
    try:
        gimbal2ros_node.run()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
