#UAVInfo.msg

#
std_msgs/Header                header

#
int32                          id
int32                          group_id
geometry_msgs/PoseStamped      pos
geometry_msgs/TwistStamped     vel
std_msgs/Float32MultiArray     adjacency_mat        # 单行表示所有, 头尾相接
std_msgs/Float32MultiArray     dist
int32                          tracking_tgt
int32                          mission_stat
float64                        lat
float64                        lon