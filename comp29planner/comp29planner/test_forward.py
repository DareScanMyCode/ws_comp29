import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ned = self.create_publisher(TwistStamped, '/vel_set_ned', 1)
        self.publisher_frd = self.create_publisher(TwistStamped, '/vel_set_frd', 1)
        # timer_period = 0.1  # 发布周期（秒）
        # self.timer = self.create_timer(timer_period, self.publish_velocity_ned)

    def publish_velocity_ned(self, x=0.0, y=0.0):
        msg = TwistStamped()
        
        # 设置消息头部
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 设置线速度
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = 0.0
        
        # 设置角速度
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_ned.publish(msg)
        self.get_logger().info(f"Publishing NED: x:{x}, y:{y}")
        
    def publish_velocity_frd(self, x=0.0, y=0.0):
        msg = TwistStamped()
        
        # 设置消息头部
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 设置线速度
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = 0.0
        
        # 设置角速度
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_frd.publish(msg)
        self.get_logger().info(f"Publishing FRD: x:{x}, y:{y}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    # rclpy.spin(node)
    try:
        input("NED向北1.5？")
        node.publish_velocity_ned(1.5, 0.0)
        
        input("暂停？")
        node.publish_velocity_ned(0.0, 0.0)
        
        
        input("NED向北1.5 东0.5？")
        node.publish_velocity_ned(1.5, 0.5)
        
        input("暂停？")
        node.publish_velocity_ned(0.0, 0.0)
    
        
        input("NED向南1.5 西0.5？")
        node.publish_velocity_ned(-1.5, -0.5)
        
        input("暂停？")
        node.publish_velocity_ned(0.0, 0.0)
        
        input("FRD向前1.5？")
        node.publish_velocity_frd(1.5, 0.0)
        
        input("暂停？")
        node.publish_velocity_frd(0.0, 0.0)
        
        input("NED向前1.5 右0.5？")
        node.publish_velocity_frd(1.5, 0.5)
        
        input("暂停？")
        node.publish_velocity_frd(0.0, 0.0)
        
        input("NED向后1.5 左0.5？")
        node.publish_velocity_frd(-1.5, -0.5)
    except KeyboardInterrupt:
        node.publish_velocity_frd(0.0, 0.0)
        pass
    input("结束？")
    node.publish_velocity_frd(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
