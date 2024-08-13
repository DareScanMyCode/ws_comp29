# from .gport_sdk.goprt import GportGimbal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from quaternions import Quaternion as Quaternion
import math
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
import numpy as np
from scipy.optimize import minimize

from sensor_msgs.msg        import Imu
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import Float32MultiArray
from quaternions            import Quaternion as Quaternion
from geometry_msgs.msg      import TwistStamped
from std_msgs.msg           import Int64, Float64
from comp29planner.utils    import *

class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []
        self.result = 0

    def add_sample(self, new_sample):
        self.data.append(new_sample)
        # 保持数据窗口不超过窗口大小
        if len(self.data) > self.window_size:
            self.data.pop(0)
        self.result = self.__get_average()
        

    def get_value(self, new_sample = None):
        if new_sample is not None:
            self.add_sample(new_sample)
        return self.result
    
    def __get_average(self):
        return sum(self.data) / len(self.data) if len(self.data) != 0 else 0
    
class RendNode(Node):
    def __init__(self, args):
        super().__init__('test_rend')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
        )
        self.guard_pos_est_ned      = self.create_publisher(PoseStamped,                '/guard_pos_est_ned', qos_profile)
        
        # 创建QoS配置文件
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            # history=QoSHistoryPolicy.SYSTEM_DEFAULT,         # 只保留最新的历史消息
            durability=DurabilityPolicy.VOLATILE,

            depth=1                                    # 历史消息的队列长度
        )
        self.local_pos = [0., 0.]
        self.local_pos_int = [0., 0.]
        self.local_pos_fcu = [0., 0.]
        
        self.local_pos_dists = []
        self.dist = 0
        self.dist_got = False
        self.pos_got = False
        self.fcu_pos_got = False
        self.uav_id = int(os.environ.get('UAV_ID', '-1'))
        if self.uav_id == -1:
            self.get_logger.warn("[rend ctrler]UAV_ID未被设置！！！！")

        self.exit_flag = False
        if self.uav_id == rend_tgt[self.uav_id]:
            self.exit_flag = True
        self.opt_pos_x_filter = MovingAverageFilter()
        self.opt_pos_y_filter = MovingAverageFilter()
        
        ekf2_pos_topic_name = '/uav' + str(self.uav_id) + "/ekf2/pose"
        
        # 创建订阅者并应用QoS配置
        self.local_pos_ned_est_sub        = self.create_subscription(
            PoseStamped, 
            # TODO 换成local_pos_est
            ekf2_pos_topic_name, 
            self.local_pos_ned_est_cb, 
            qos_profile
        )
        
        self.local_pos_ned_fcu_sub        = self.create_subscription(
            PoseStamped, 
            # TODO 换成local_pos_est
            '/local_position_ned', 
            self.local_pos_ned_fcu_cb, 
            qos_profile
        )
        
        self.dist_sub        = self.create_subscription(
            Float32MultiArray, 
            '/uwb_filtered', 
            self.dist_cb, 
            qos_profile
        )
        
        self.vel_ned_sub        = self.create_subscription(
            TwistStamped, 
            '/velocity_and_angular', 
            self.vel_cb, 
            qos_profile
        )
        print("ready")
    
    def vel_cb(self, msg: TwistStamped):
        self.vel = [msg.twist.linear.x, msg.twist.linear.y]
        self.vel_got = True
        
    def dist_cb(self, msg:Float32MultiArray):
        # print(msg)
        self.dist = msg.data[self.uav_id*msg.layout.dim[0].size + rend_tgt[self.uav_id]]
        self.dist_got = True
        # print(11)
        pass
    
    def local_pos_ned_est_cb(self, msg: PoseStamped):
        self.local_pos = [msg.pose.position.x, msg.pose.position.y]
        self.pos_got = True
        # print(22)
        pass
    
    def local_pos_ned_fcu_cb(self, msg: PoseStamped):
        self.local_pos_fcu = [msg.pose.position.x, msg.pose.position.y]
        self.fcu_pos_got = True
        # print(22)
        pass
    
    def solve(self):
        pass
    
    
    
    def run(self):
        while True:
            rclpy.spin_once(self)
            if self.pos_got and self.dist_got:
                break
            else:
                print("waiting for msgs")
                time.sleep(1)
        p_star = np.array([0, 0])
        p_opt = np.array([5,5])
        p = np.array([0, 0])
        r_hat = np.array([5, 0])
        vv = r_hat / np.linalg.norm(r_hat)
        p_hat = p.copy()
        MAX_SIZE = 3000
        TIME_STEP = 0.1
        his_p = np.zeros((MAX_SIZE, 2))
        his_p_hat = np.zeros((MAX_SIZE, 2))
        his_d = np.zeros(MAX_SIZE)
        his_p[0, :] = self.local_pos
        his_p_hat[0, :] = p_hat
        his_d[0] = np.linalg.norm(p - p_star)
        p_opts = np.zeros((MAX_SIZE, 2))
        step = -1
        window = 10
        while True:
            
            step+=1
            self.local_pos_dists.append([self.local_pos[0], self.local_pos[1], self.dist])
            if len(self.local_pos_dists) < 14:
                rclpy.spin_once(self)
                time.sleep(0.1)
                continue
            if math.fabs(self.local_pos_dists[-1][0]) < 0.1 and math.fabs(self.local_pos_dists[-1][1] < 0.1) and math.fabs(self.local_pos_dists[-1][0] - self.local_pos_dists[-3][0]) > 0.4 and math.fabs(self.local_pos_dists[-1][1] - self.local_pos_dists[-3][1]) > 0.4:
                print("========================================================")
                print("==== LOCAL POSITION NED 已被飞控重置，尝试重置历史数据 ===")
                print("========================================================")
                self.local_pos_dists = []
                continue
            if len(self.local_pos_dists) > 200:
                step = 99
                # 随机取100个保留
                indexes = np.abs(np.random.randn(100) * 199).astype(int)
                indexes = np.clip(indexes, 0, 199)
                self.local_pos_dists = [self.local_pos_dists[i] for i in indexes]
            
            if self.local_pos_dists[-1][2] < self.local_pos_dists[-2][2] and np.linalg.norm(np.dot(vv, TIME_STEP) - (np.array(self.local_pos_dists[-2][2]) - np.array(self.local_pos_dists[-1][0:2]))) / np.linalg.norm(vv) / TIME_STEP < 0.05:
                print("似乎朝着正确方向")
                p_opts[step][:] = p_opt
                continue
            # 随机采样
            indexes = np.abs(np.random.randn(window) * step).astype(int)
            indexes = np.clip(indexes, 0, step)
            # print(indexes)
            
            # print(self.local_pos_dists)
            hat_p = [self.local_pos_dists[i][0:2] for i in indexes]
            d = [self.local_pos_dists[i][2] for i in indexes]
            
            # 定义目标函数
            def objective_function(p):
                return np.sum([(np.linalg.norm(hat_p[i][:] - p) - d[i]) ** 2 for i in range(window)])
            
            # 初始猜测 p0
            p0 = p_opt
            
            # 使用 scipy.optimize.minimize 求解优化问题
            res = minimize(objective_function, p0, method='BFGS', options={'disp': False})
            p_opt = res.x
            p_opts[step, :] = p_opt
            
            pos_opt_msg = PoseStamped()
            pos_opt_msg.position.x = self.opt_pos_x_filter.get_value(p_opt[0])
            pos_opt_msg.position.y = self.opt_pos_y_filter.get_value(p_opt[1])
            self.guard_pos_est_ned.publish(pos_opt_msg)
            rclpy.spin_once(self)
            time.sleep(0.1)
            

def main(args=None):
    rclpy.init()
    
    rend_node = RendNode(args)
    
    try:
        if rclpy.ok() and not rend_node.exit_flag:
            rend_node.run()
        else:
            rend_node.get_logger().info(f"{ColorCodes.red}[rend] 测量位置目标为自身，进程结束{ColorCodes.reset}")
    finally:
        print("REND 线程结束")
        # rclpy.shutdown()

if __name__ == "__main__":
    main()
