import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

rclpy.init()
node = Node('test_node')
qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            # history=QoSHistoryPolicy.SYSTEM_DEFAULT,         # 只保留最新的历史消息
            durability=DurabilityPolicy.VOLATILE,

            depth=1                                    # 历史消息的队列长度
        )
pub = node.create_publisher(PoseStamped, '/gimbalrpy_setpoint', qos_profile)

while True:
    try:
        pitch = input("请输入pitch：")
        pitch = float(pitch)
        msg = PoseStamped()
        msg.pose.position.y = pitch
        pub.publish(msg)
    except Exception as e:
        print(e)
        break
