import yaml
import numpy as np
#import rospy
import random
import struct
#
import os
import time
import functools
import schedule
from colorama import Fore, Back, Style
#
from comp29communicator.fsm import fsm
from comp29communicator.flow_monitor import getNetworkRate
import comp29communicator.fsm_command
from comp29communicator.UDPProcess import UDPCommunicator
#
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from comp29msg.msg import UAVInfo, CommunicationInfo                        # TODO Remember to ``source ./install/setup.bash''

'''
  parameters
'''
uav_id = 3                                      # 改这里是没意义的, 要改yaml
uav_group_id = 1
swarm_num = 8
known_target_num = 3
#
target_ip_for_debugging = '127.0.0.1'           # 单引号
target_port = 25001
ip_list = []
#
is_debugging = False

this_uav_pos = PoseStamped()
is_pos_updated = False
def pos_cb(msg):
    global this_uav_pos, is_pos_updated
    #
    this_uav_pos = msg
    is_pos_updated = True

this_uav_vel = TwistStamped()
is_vel_updated = False
def vel_cb(msg):
    global this_uav_vel, is_vel_updated
    #
    this_uav_vel = msg
    is_vel_updated = True

planner_info = UAVInfo()
is_uav_info_updated = False
def planner_info_cb(msg):
    global planner_info, is_uav_info_updated
    #
    planner_info = msg
    is_uav_info_updated = True

uwb_data = Float32MultiArray()
is_uwb_updated = False
def uwb_cb(msg):
    global uwb_data, is_uwb_updated
    #
    uwb_data = msg
    is_uwb_updated = True

gps_data = PoseStamped()
is_gps_updated = False
def gps_cb(msg:PoseStamped):
    global gps_data, is_gps_updated
    #
    gps_data = msg
    is_gps_updated = True

class rcl_fsm(fsm):
    def __init__(self, idd, swarm_num, known_target_num, group, ip_list, target_port=25001):
        super(rcl_fsm, self).__init__(idd, swarm_num, known_target_num, group, target_port)

        self.dist_mat = np.zeros([self.swarm_num, self.swarm_num])      # 有权邻接矩阵, 权重为距离

        self.ip_list = []                                               # 发送对象
        for i in range(0, ip_list.__len__()):
            self.ip_list.append(ip_list[i])
        print("[fsm] ip list:")
        print(self.ip_list)

        #
        # ROS接口
        self.swarm_info = CommunicationInfo()
        for i in range(0, self.swarm_num):
            #
            self.swarm_info.uav.append(UAVInfo())
            #
            for j in range(0, self.swarm_num):
                 self.swarm_info.uav[i].dist.data.append(0)
            #
            for j in range(0, self.swarm_num):
                 for k in range(0, self.swarm_num):
                      self.swarm_info.uav[i].adjacency_mat.data.append(0)
        #
        # common info
        for i in range(0, self.known_target_num):
            for j in range(0, 3):
                self.swarm_info.tracking_list.data.append(0)
        for i in range(0, self.known_target_num):
            for j in range(0, 2):
                self.swarm_info.guard_list.data.append(0)

        # 
        # dist mat
        for i in range(0, self.swarm_num):
                for j in range(0, self.swarm_num):
                    self.swarm_info.dist_mat.data.append(0)
 
        self.is_data_received = False
        #
        self.ack_companion = False
        self.ack_cmd_id = -1


    def FSMCallback(self):
        '''
        state machine
        '''
        self.is_send_info = True
        if self.is_send_info:
            data_t = self.Pack_Info(is_debugging=False)                                                     # TODO debugging = False, 这个必须改不然消息是错的
            for i in range(0, self.ip_list.__len__()):
                self.udpc.send(data_t, ip_list[i], target_port)
                #if self.is_debugging:
                if False:
                    print("[fsm] sending to " + ip_list[i] + "...")
                time.sleep(0.005)
            #if self.is_debugging:
            if False:
                # debugging
                data_recv_t = self.Unpack_Info(data_t, is_debugging=self.is_debugging)

        if self.is_send_critical_info:
            #
            data_t = self.Pack_Critical_Status(cmd_id=fsm_command.kill, is_debugging=False)                 # CHECKED debugging = false
            for i in range(0, self.ip_list.__len__()):
                self.udpc.send(data_t, ip_list[i], target_port)
                time.sleep(0.005)
            #if self.is_debugging:
            if False:
                # debugging
                data_recv_t = self.Unpack_Info(data_t, is_debugging=self.is_debugging)


        if self.is_send_critical_info_ack:
            data_t = self.Pack_Critical_Status_Ack(companion_id=self.ack_companion, cmd_id=self.ack_cmd_id, is_debugging=False)        # CHECKED debugging = false
            for i in range(0, self.ip_list.__len__()):
                self.udpc.send(data_t, ip_list[i], target_port)
                time.sleep(0.005)
            #if self.is_debugging:            
            if False:
                # debugging
                data_recv_t = self.Unpack_Info(data_t, is_debugging=self.is_debugging)

        #
        if self.udpc.recv_data_list.__len__():
            data_num_to_handle = 5
            len_t = min(data_num_to_handle, self.udpc.recv_data_list.__len__())     # 一次考虑只处理几条
            for i in range(0, len_t):
                data_addr_t = self.udpc.recv_data_list.pop(-1)
                data_recv_t = self.Unpack_Info(data_addr_t[0], is_debugging=self.is_debugging)

                self.update_swarm_info(data_recv_t)

                if False:
                    print("[fsm] udp received: ")
                    print(data_addr_t)

            self.is_data_received = True


        if self.is_send_info:
            self.is_send_info = False
        if self.is_send_critical_info:
            self.is_send_critical_info = False
        if self.is_send_critical_info_ack:
            self.is_send_critical_info_ack = False

        print("233")

    def update_swarm_info(self, data_recv):

        if data_recv['frame_id'] == 1:
            id_t = data_recv['uav_id']
            #
            self.swarm_info.uav[id_t].id = id_t
            self.swarm_info.uav[id_t].group_id = data_recv['group_id']
            #
            self.swarm_info.uav[id_t].pos.pose.position.x = data_recv['pos'][0]
            self.swarm_info.uav[id_t].pos.pose.position.y = data_recv['pos'][1]
            self.swarm_info.uav[id_t].pos.pose.position.z = data_recv['pos'][2]            
            #
            self.swarm_info.uav[id_t].vel.twist.linear.x = data_recv['vel'][0]
            self.swarm_info.uav[id_t].vel.twist.linear.y = data_recv['vel'][1]
            self.swarm_info.uav[id_t].vel.twist.linear.z = data_recv['vel'][2]
            self.swarm_info.uav[id_t].lat = data_recv['lat']
            self.swarm_info.uav[id_t].lon = data_recv['lon']
            #
            for i in range(0, self.swarm_num):
                 self.swarm_info.uav[id_t].dist.data[i] = data_recv['dist_list'][i]
            #
            for i in range(0, self.swarm_num):
                 for j in range(0, self.swarm_num):
                      self.swarm_info.uav[id_t].adjacency_mat.data[i * self.swarm_num + j] = self.adjacency_mat[i, j]
            #        
            self.swarm_info.uav[id_t].tracking_tgt = data_recv['current_tgt']
            self.swarm_info.uav[id_t].mission_stat = data_recv['mission_stat']

            #
            # target_list
            self.tgt_list.sort(key=functools.cmp_to_key(self.sort_tgt_list_guard_list), reverse=True)      # 做了这一步以后, identified, i.e., -1, 一定在最后
            #if self.is_debugging:
            if False:
                print(self.tgt_list)
            #
            is_found_current_id = False
            for i in range(0, self.known_target_num):
                #
                tgt_id_updated = data_recv['tgt_list'][i][0]                    # 读入的id
                x_new = data_recv['tgt_list'][i][1]
                y_new = data_recv['tgt_list'][i][2]
                if tgt_id_updated == -1:
                    continue
                # 遍历已发现目标
                for j in range(0, self.known_target_num):
                    target_id = self.tgt_list[j][0]
                    if tgt_id_updated == target_id:
                        alpha = 0.33
                        # x
                        self.tgt_list[j][1] = (1. - alpha) * self.tgt_list[j][1] + alpha * x_new            # lpf
                        # y
                        self.tgt_list[j][2] = (1. - alpha) * self.tgt_list[j][2] + alpha * y_new
                        is_found_current_id = True
                    elif not is_found_current_id and target_id == -1:           # 找到最后发现没有找到当前id, list还有空
                        self.tgt_list[j][0] = tgt_id_updated
                        self.tgt_list[j][1] = x_new
                        self.tgt_list[j][2] = y_new                             # 直接更新
            # 
            for i in range(0, self.known_target_num):
                self.swarm_info.tracking_list.data[i * 3]     = self.tgt_list[i][0]
                self.swarm_info.tracking_list.data[i * 3 + 1] = self.tgt_list[i][1]
                self.swarm_info.tracking_list.data[i * 3 + 2] = self.tgt_list[i][2]

            #
            # guard list
            self.guard_list.sort(key=functools.cmp_to_key(self.sort_tgt_list_guard_list), reverse=True)
            is_found_current_id = False
            for i in range(0, self.known_target_num):
                #
                tgt_id_updated = data_recv['guard_list'][i][0]                  # 读入的id
                x_new = data_recv['guard_list'][i][1]
                if tgt_id_updated == -1:
                    continue
                # 遍历已发现目标
                for j in range(0, self.known_target_num):
                    target_id = self.guard_list[j][0]
                    if tgt_id_updated == target_id:
                        is_found_current_id = True
                    elif not is_found_current_id and target_id == -1:
                        self.guard_list[j][0] = tgt_id_updated
                        self.guard_list[j][1] = id_t                              # 如果当前的目标没有飞机守着, 则更新守着的飞机
            #
            for i in range(0, self.known_target_num):
                self.swarm_info.guard_list.data[i * 2]     = self.guard_list[i][0]
                self.swarm_info.guard_list.data[i * 2 + 1] = self.guard_list[i][1]

            #
            self.update_dist_mat(data_recv)
            #
            for i in range(0, self.swarm_num):
                 for j in range(0, self.swarm_num):
                      self.swarm_info.dist_mat.data[i * self.swarm_num + j] = self.adjacency_mat[i, j]
            #
            this_uav_data = self.update_this_uav_data()
            self.swarm_info.uav.append(this_uav_data)
            self.swarm_info.uav.sort(key=functools.cmp_to_key(self.sort_swarm_info), reverse=True)      # 不加自己的序号会错
            
            if is_debugging:
            #if True:
                print(self.swarm_info.uav)

            
        elif data_recv['frame_id'] == 2:
            #
            companion_id_t = data_recv['id']
            self.swarm_info.uav[companion_id_t].mission_stat = data_recv['cmd_id']
        elif data_recv['frame_id'] == 3:
            #
            self.ack_companion = data_recv['companion_id']
            self.ack_cmd_id    = data_recv['cmd_id']
            #
            self.is_send_critical_info_ack = True

    def update_dist_mat(self, data_recv):

        if 'dist_list' not in data_recv.keys():
            return
        #
        companion_id = data_recv['uav_id']
        dist_list_t = data_recv['dist_list']
        #
        for i in range(0, self.swarm_num):
            # TODO
            # 极大极小值滤除
            self.dist_mat[companion_id, i] = dist_list_t[i]

    def update_this_uav_data(self):
        this_uav_data = UAVInfo()
        this_uav_data.id = self.id
        this_uav_data.group_id = self.group
        #
        this_uav_data.pos.pose.position.x = self.pos_xyz[0]
        this_uav_data.pos.pose.position.y = self.pos_xyz[1]
        this_uav_data.pos.pose.position.z = self.pos_xyz[2]
        #
        this_uav_data.vel.twist.linear.x = self.vel_xyz[0]
        this_uav_data.vel.twist.linear.y = self.vel_xyz[1]
        this_uav_data.vel.twist.linear.z = self.vel_xyz[2]
        this_uav_data.lat = self.lat
        this_uav_data.lon = self.lon
        #
        for i in range(0, self.swarm_num):
                for j in range(0, self.swarm_num):
                    #this_uav_data.adjacency_mat.data[i * self.swarm_num + j] = self.dist_mat[i, j]
                    this_uav_data.adjacency_mat.data.append(self.dist_mat[i, j])
        #
        for i in range(0, self.swarm_num):
            #this_uav_data.dist.data[i] = self.dist_mat[self.id, i]
            this_uav_data.dist.data.append(self.dist_mat[self.id, i])
        #
        this_uav_data.tracking_tgt = self.current_tgt
        this_uav_data.mission_stat = self.mission_status

        return this_uav_data


    def sort_swarm_info(self, x:UAVInfo, y:UAVInfo):
        '''
            usage:
            strs.sort(key=functools.cmp_to_key(my_compare), reverse=True)
        '''
        if x.id > y.id:
            return 1
        elif x.id < y.id:
            return -1
        return 0

def network_rate_thread(args=None):
    _, networkIn, networkOut = getNetworkRate(1)                # num -> 时间

    print(Style.BRIGHT + Fore.BLUE + "[fsm flow monitor] In_rate: ", networkIn, "    Out_rate: ", networkOut)
    print (Style.RESET_ALL)


def main(args=None):

    global uav_id, uav_group_id, swarm_num, known_target_num
    global ip_list, target_port
    global is_debugging
    #
    global this_uav_pos, is_pos_updated
    global this_uav_vel, is_vel_updated
    global planner_info, is_uav_info_updated        
    global uwb_data, is_uwb_updated
    global gps_data, is_gps_updated

    rclpy.init(args=args)
    node = Node('Communicator_FSM')
    username = os.getenv("USER", "cat")
    if username == 'cat':
        comm_table_dir = "/home/cat/ws_comp29/src/comp29communicator/comp29communicator/data/comm_table.yaml"
    elif username == 'zbw':
        comm_table_dir = "/home/zbw/ws_comp29/src/comp29communicator/comp29communicator/data/comm_table.yaml"
        
    else:
        comm_table_dir = "/mnt/d/ubuntu_wsl/colcon_ws/src/comp29communicator/comp29communicator/data/comm_table.yaml"
    with open(comm_table_dir, 'r', encoding='utf-8') as f:
        comm_table_t = yaml.load(f.read(), Loader=yaml.FullLoader)
    
    if is_debugging:
        node.get_logger().info(f"{comm_table_t}")

    # 参数
    uav_id = int(os.environ.get('UAV_ID', '-1'))
    if uav_id == -1:
        node.get_logger().warn("[fsm] UAV_ID未被设置！！！！")
    username = os.environ.get("USER", 'cat')
    filepath = f'/home/{username}/ws_comp29/src/configs/missionCFG.json'
    # 读取任务配置文件
    if not os.path.exists(filepath):
        node.get_logger().error(f"[fsm] Config file {filepath} does not exist.")
        return None
    # 打开并读取 JSON 文件
    import json
    try:
        with open(filepath, 'r') as file:
            mis_cfg = json.load(file)
        node.get_logger().info(f"Config file {filepath} loaded successfully.")
    except Exception as e:
        node.get_logger().error(f"Failed to read config file: {e}")
    
    if mis_cfg is not None:
        map_w              = mis_cfg['MAP_W']
        map_l              = mis_cfg['MAP_L']
        map_angle          = mis_cfg['MAP_ANGLE']
        dx_per_line        = mis_cfg['DX_PER_LINE']
        
        leader_id          = mis_cfg['LEADER_ID']
        angle_leader_id    = mis_cfg['ANGLE_LEADER_ID']
        swarm_num            = mis_cfg['NUM_UAV']
        
        group_1            = mis_cfg['GROUP_1']
        group_2            = mis_cfg['GROUP_2']
        group_3            = mis_cfg['GROUP_3']
        
        uav_group_id = 1 if uav_id in group_1 else 2 if uav_id in group_2 else 3 if uav_id in group_3 else -1
        
        if uav_group_id == -1:
            node.get_logger().warn("[fsm] UAV_ID未被分组！！！！")
            
        port = mis_cfg['COMM_PORT']
        known_target_num = mis_cfg['TARGET_NUM']
            
    else:
        node.get_logger().warn("[fsm] 未能读取到任务配置文件！！！！")
        
        
    # uav_group_id  = comm_table_t['group_id']
    # swarm_num = comm_table_t['swarm_num']
    # known_target_num = comm_table_t['known_tgt_num']
    # port = comm_table_t['port']
    for i in range(0, comm_table_t['target_ip'].__len__()):
        ip_t = comm_table_t['target_ip'][i]
        #if ip_t == comm_table_t['self_ip'] and not is_debugging:
        if False:
            continue                # updated
        ip_list.append(ip_t)

    fsm_t = rcl_fsm(uav_id, swarm_num, known_target_num, uav_group_id, ip_list, target_port=target_port)


    pos_topic_name = "/uav" + str(uav_id) + "/ekf2/pose"               #               'local_position_ned'                '/uav' + str(uav_id) + "/ekf2/pose"
    vel_topic_name = "/uav" + str(uav_id) + "/ekf2/vel"                # from MAVLink: 'velocity_and_angular'   from EKF2: '/uav' + str(uav_id) + "/ekf2/vel"
    uav_info_topic_name  = "/uav" + str(uav_id) + "/planner_info"
    comm_info_topic_name = "/uav" + str(uav_id) + "/comm_info"
    uwb_topic_name       = 'uwb_filtered_topic'
    gps_topic_name       = 'gps_position'
    #
    position_sub  = node.create_subscription(PoseStamped,  pos_topic_name, pos_cb, 1)
    velocity_sub  = node.create_subscription(TwistStamped, vel_topic_name, vel_cb, 1)
    uav_info_sub  = node.create_subscription(UAVInfo,      uav_info_topic_name, planner_info_cb, 1)
    uwb_info_sub  = node.create_subscription(Float32MultiArray, uwb_topic_name, uwb_cb, 1)
    gps_pos_sub   = node.create_subscription(PoseStamped, gps_topic_name, gps_cb, 1)
    #
    comm_info_pub = node.create_publisher(CommunicationInfo,  comm_info_topic_name, 1)

    try:
        schedule.every(0.1).seconds.do(fsm_t.FSMCallback)                 # TODO, 调短
        schedule.every(0.05).seconds.do(network_rate_thread)

        fsm_t.udpc.start_receive_thread()

        while True:  
            # 
            # ROS收到Planner的数据转发
            if is_uav_info_updated:
                fsm_t.update(current_tgt=planner_info.tracking_tgt, mission_stat=planner_info.mission_stat)

                is_uav_info_updated = False

            # 
            # ROS收到Localization的数据转发
            if is_pos_updated:
                fsm_t.update(pos=[this_uav_pos.pose.position.x, this_uav_pos.pose.position.y, this_uav_pos.pose.position.z])

                is_pos_updated = False

            # 
            # ROS收到Localization的数据转发
            if is_vel_updated:
                fsm_t.update(vel=[this_uav_vel.twist.linear.x, this_uav_vel.twist.linear.y, this_uav_vel.twist.linear.z])

                is_vel_updated = False

            # TODO
            # ROS收到UWB数据转发
            if is_uwb_updated:
                fsm_t.update(dist_list=uwb_data.data)

                is_uwb_updated = False

            # ROS收到GPS数据转发
            if is_gps_updated:
                fsm_t.update(latlon=[gps_data.pose.position.x, gps_data.pose.position.y])

                is_gps_updated = False

            # 发送新消息
            if fsm_t.is_data_received:
                fsm_t.swarm_info.header.stamp = node.get_clock().now().to_msg()
                comm_info_pub.publish(fsm_t.swarm_info)

                fsm_t.is_data_received = False

            schedule.run_pending()                                       # 运行待执行的任务队列
            time.sleep(0.005)                                            # 这个刷新速度要小于scheduled task的
    except KeyboardInterrupt:
        node.get_logger().info("[fsm] 服务器正在关闭...")
    finally:
        fsm_t.shutdown()

    node.get_logger().info("[fsm] all stopped ...")

if __name__ == "__main__":
    main()
