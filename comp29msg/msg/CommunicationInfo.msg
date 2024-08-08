# CommunicationInfo.msg

# Header for timestamp and frame id
std_msgs/Header header

#
UAVInfo[] uav
# 目标位置
std_msgs/Float32MultiArray tracking_list        # 单行表示所有
# 哪个目标由哪架飞机守着
std_msgs/Float32MultiArray guard_list           # 单行表示