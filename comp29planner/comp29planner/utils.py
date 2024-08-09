class MissionState:
    MIS_WAIT                    = 0
    MIS_FORMATION_LEADER        = 10
    MIS_FORMATION_FOLLOW        = 11
    MIS_FORMATION_LEAVING       = 12
    MIS_BASE                    = 20
    MIS_BASE_STAND              = 21
    MIS_BASE_MOVE               = 22
    MIS_TARGET_WATCHING         = 30
    MIS_LANDING                 = 40


class AgentRole:
    ROLE_LEADER                 = 1     # leader
    ROLE_ANGLE_LEADER           = 2     # 第二leader
    ROLE_FOLLOWER               = 3     # Follower
    ROLE_BASE                   = 10    # 基站

class DetectedResult:
    def __init__(self) -> None:
        self.detected_num = 0
        self.confidence = 0.
        self.detected_pose = []
        self.detected_size = []
        pass
from comp29msg.msg import DetectionArray, DetectionResult
"""
# DetectionResult.msg
std_msgs/Header header
std_msgs/Int64 object_name
std_msgs/Float32 confidence

# central position
geometry_msgs/Point position

# x_size y_size angle
geometry_msgs/Vector3 size 
===============================
# DetectionArray.msg

# Header for timestamp and frame id
std_msgs/Header header

# Array of detection results
DetectionResult[] detections

"""
class DetectedResultList:
    def __init__(self) -> None:
        
        self.detected_results = []
        pass
    
    def msg2result(self, msg):
        self.detected_results = []
        for rst in msg.detections:
            detected_result = DetectedResult()
            detected_result.detected_num = int(rst.object_name)
            detected_result.confidence = float(rst.confidence)
            detected_result.detected_pose = [rst.detected_pose.x, rst.detected_pose.y]
            detected_result.detected_size = [rst.detected_size.x, rst.detected_size.y]
            self.detected_results.append(detected_result)
            pass
        pass
    
color_codes = {
            'red': '\033[31m',
            'green': '\033[32m',
            'yellow': '\033[33m',
            'blue': '\033[34m',
            'magenta': '\033[35m',
            'cyan': '\033[36m',
            'white': '\033[37m',
            'reset': '\033[0m'
        }