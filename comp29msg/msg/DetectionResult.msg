# DetectionResult.msg
std_msgs/Header header
std_msgs/Int64 object_name
std_msgs/Float32 confidence

# central position
geometry_msgs/Point position

# x_size y_size angle
geometry_msgs/Vector3 size 