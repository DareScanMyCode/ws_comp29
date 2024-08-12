#!/usr/bin/python
# -*- coding: utf-8 -*-
from  comp29communicator.UDPProcess import UDPCommunicator
import numpy as np
#import rospy
import random
import struct
#
import math
import time
import functools
import schedule
import  comp29communicator.fsm_command

'''
  parameters
'''
uav_id = 3
uav_group_id = 1
# swarm_num = 8
known_target_num = 3
#
target_ip_for_debugging = '127.0.0.1'           # 单引号
target_port = 25001

class fsm():
    def __init__(self, idd, swarm_num, known_target_num, group, target_port=25001):
        self.id=idd #priority
        self.state = 'init'
        self.state_list = ['init', 'takeoff','hover','land','Station']
        self.state_idx = 0
        self.state = self.state_list[self.state_idx]
        self.group = group
        self.swarm_num = swarm_num

        self.target_port = target_port
        self.udpc=UDPCommunicator(self.target_port)#这里面存储了数据

        self.swarm_active_list=np.zeros(swarm_num)
        self.swarm_states_list=np.zeros(swarm_num)        # 0：standby 1:station
        self.swarm_pos_list=np.zeros([swarm_num,3])       #
        self.swarm_group_list=np.zeros(swarm_num)         # 1:group1 2:group2,3:group3

        self.pos_xyz = [0., 0., 0.]
        self.vel_xyz = [0., 0., 0.]
        self.lat = 0.
        self.lon = 0.

        #
        # -1数据有问题
        #  0不联通
        self.neighbor_dist_list = []
        for i in range(0, swarm_num):
            self.neighbor_dist_list.append(-1.)
        self.adjacency_mat = np.zeros([swarm_num, swarm_num])

        #
        # 
        # id为空表示没找到, 位置自然无效, id有的话表示找到
        # [
        #  [id1, x, y]
        #  [id2, x, y]
        #  [id3, x, y]
        # ]
        self.known_target_num = known_target_num
        self.tgt_list = []
        for i in range(0, known_target_num):
            self.tgt_list.append([-1, 0, 0])

        #
        # 哪个飞机守着哪个目标
        # [
        #  [target_id1, uav_id_a]
        #  [target_id2, uav_id_b]
        #  [target_id3, uav_id_c]
        # ]
        self.guard_list = []
        for i in range(0, known_target_num):
            self.guard_list.append([-1, 0])

        #
        #
        self.current_tgt = -1
        self.mission_status = -1 

        self.fsm_state='no_msg'
        self.fsm_list=['no_msg','tag','bind','trajectory','heartbeat']
        self.last_hearbeat_time=time.time()

        #
        self.is_send_info = False
        self.is_send_critical_info = False
        self.is_send_critical_info_ack = False

        # 放最后
        self.is_debugging = False
        self.t_start = time.time()

    def shutdown(self):
        self.udpc.close()
    
    
    def FSMCallback(self):#timer 循环调用，判断不同的状态执行不通过操作
        '''
        state machine
        '''
        self.is_send_info = True
        if self.is_send_info:
            data_t = self.Pack_Info(is_debugging=False)                                                     # CHECKED debugging = false, 这个必须改不然消息是错的
            self.udpc.send(data_t, '127.0.0.1', target_port)
            #if self.is_debugging:
            if False:
                # debugging
                data_recv_t = self.Unpack_Info(data_t, is_debugging=self.is_debugging)

        if self.is_send_critical_info:
            #
            data_t = self.Pack_Critical_Status(cmd_id=fsm_command.kill, is_debugging=False)                 # CHECKED debugging = false
            self.udpc.send(data_t, '127.0.0.1', target_port)
            #if self.is_debugging:            
            if False:
                # debugging
                data_recv_t = self.Unpack_Info(data_t, is_debugging=self.is_debugging)


        if self.is_send_critical_info_ack:
            data_t = self.Pack_Critical_Status_Ack(companion_id=data_recv_t['uav_id'] + 2, cmd_id=data_recv_t['cmd_id'], is_debugging=False)        # CHECKED debugging = false
            self.udpc.send(data_t, '127.0.0.1', target_port)
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

                if False:
                    print("[fsm] udp received: ")
                    print(data_addr_t)


        if self.is_send_info:
            self.is_send_info = False
        if self.is_send_critical_info:
            self.is_send_critical_info = False
        if self.is_send_critical_info_ack:
            self.is_send_critical_info_ack = False


        print("233")


    def reset_uav_info(self, uav_id=None, group_id=None):
        if uav_id != None:
            self.id = uav_id
        if group_id != None:
            self.group = group_id

    def update(self, vel=None, dist_list=None, adjacency_mat=None, pos=None, latlon=None, tgt_list=None, guard_list=None, current_tgt=None, mission_stat=None):
        if vel != None:
            self.vel_xyz[0] = vel[0]
            self.vel_xyz[1] = vel[1]
            self.vel_xyz[2] = vel[2]
        if dist_list != None:
            try:
                for i in range(0, self.swarm_num):
                    self.neighbor_dist_list[i] = dist_list[i]
            except:
                print("[fsm] neighboring list updating error ...")
        if adjacency_mat != None:
            try:
                for i in range(0, self.swarm_num):
                    for j in range(0, self.swarm_num):
                        self.adjacency_mat[i, j] = adjacency_mat[i, j]
            except:
                print("[fsm] adjacency matrix updating error ...")
        if pos != None:
            self.pos_xyz[0] = pos[0]
            self.pos_xyz[1] = pos[1]
            self.pos_xyz[2] = pos[2]
        if latlon != None:
            self.lat = latlon[0]
            self.lon = latlon[1]            
        if tgt_list != None:
            try:
                for i in range(0, self.known_target_num):
                    self.tgt_list[i][0] = tgt_list[i][0]
                    self.tgt_list[i][1] = tgt_list[i][1]
                    self.tgt_list[i][2] = tgt_list[i][2]
            except:
                print("[fsm] target list updating error ...")
        if guard_list != None:
            try:
                for i in range(0, self.known_target_num):
                    self.guard_list[i][0] = guard_list[i][0]
                    self.guard_list[i][1] = guard_list[i][1]
            except:
                print("[fsm] guard list updating error ...")
        if current_tgt != None:
            self.current_tgt = current_tgt
        if mission_stat != None:
            self.mission_status = mission_stat

    def Pack_Info(self, is_debugging=False):
        # 帧头    帧头    数据帧ID  时间    飞机ID    飞机分组    速度x,y,z  邻居dist_list    邻接矩阵8*8    位置估计xy    高度估计z    lat    lon    已知目标(id + xy) * 3    已知目标跟踪飞机ID    跟踪状态    飞机任务状态 lat lon
        # 0xAB   0xEF      1
        data_to_send = bytes()
        #
        self.header1 = 0xAB                           # uint8_t
        self.header2 = 0xEF                           # uint8_t
        frame_id = 1                                  # uint8_t
        self.time_t = time.time() - self.t_start      # uint32_t *1000 -> ms
        self.id                                       # uint8_t
        self.group                                    # uint8_t
        data_to_send = struct.pack("=BBBIBB", self.header1, self.header2, frame_id, int(self.time_t * 1000), self.id, self.group)
        if is_debugging:
            print("[fsm] data_to_send 1")
            print(data_to_send)
            print(struct.unpack("BBBIBB", data_to_send))

        #
        data_to_send_2 = str()
        #
        # self.vel_xyz[0]                                # float m/s ned
        # self.vel_xyz[1]                                # float
        # self.vel_xyz[2]                                # float
        if is_debugging:
            self.vel_xyz[0] =  1.234
            self.vel_xyz[1] = -2.345
            self.vel_xyz[2] =  3.456
        data_to_send_2 = struct.pack("=fff", self.vel_xyz[0], self.vel_xyz[1], self.vel_xyz[2])
        # self.neighbor_dist_list
        for i in range(0, self.swarm_num):
            data_to_send_2 = data_to_send_2 + struct.pack("=f", self.neighbor_dist_list[i])
        #
        # CRITICAL
        data_to_send = data_to_send + data_to_send_2
        if is_debugging:
            print("[fsm] data_to_send 2")
            print(data_to_send_2)
            unpack_format_t = "=fff"
            for i in range(0, self.swarm_num):
                unpack_format_t = unpack_format_t + "f"
            print(struct.unpack(unpack_format_t, data_to_send_2))
            #
            print("[fsm] data_to_send 1 + 2")
            print(data_to_send)
            print(struct.unpack("BBBIBB", data_to_send[0:10]))
            print(struct.unpack(unpack_format_t, data_to_send[10:]))
            print(len(data_to_send))

        #
        data_to_send_3 = bytes()
        if is_debugging:
            #
            #                             小端                   大端
            self.adjacency_mat = np.matrix([[1, 0, 0, 0, 0, 0, 0, 1],
                                            [0, 1, 0, 0, 0, 0, 0, 0],
                                            [0, 0, 1, 0, 0, 0, 0, 0],
                                            [0, 0, 0, 1, 0, 1, 1, 1],
                                            [0, 0, 0, 0, 1, 0, 0, 0],
                                            [0, 0, 0, 0, 0, 1, 0, 0],
                                            [0, 0, 0, 0, 0, 0, 1, 0],
                                            [0, 1, 0, 0, 0, 0, 0, 1],])
            self.pos_xyz[0] =  5.678
            self.pos_xyz[1] = -6.789
            self.pos_xyz[2] =  7.890
            self.lat        =  24.123456789
            self.lon        = -118.98765432
        # 邻接矩阵位运算
        # adjacency_mat_to_send = []              # 一个int8表示一行,
        for i in range(0, self.swarm_num):
            mat_line_t = 0
            for j in range(0, self.swarm_num):
                if self.adjacency_mat[i, j] == 1:
                    mat_line_t = mat_line_t | (1 << j)
            # adjacency_mat_to_send.append(mat_line_t)
            data_to_send_3 = data_to_send_3 + struct.pack("=B", mat_line_t)
        #
        # self.pos_xyz[0]
        # self.pos_xyz[1]
        # self.pos_xyz[2]
        # self.lat              float * 1.e7 deg 这里必须这么发(连着发f)不然打包格式会有问题
        # self.lat 小数位       float * 1.7e deg 
        # self.lon              float * 1.e7 deg 这里必须这么发(连着发f)不然打包格式会有问题
        # self.lon 小数位       float * 1.7e deg 
        data_to_send_3 = data_to_send_3 + struct.pack("=fff", self.pos_xyz[0], self.pos_xyz[1], self.pos_xyz[2])
        data_to_send_3 = data_to_send_3 + struct.pack("=ffff",  float(self.lat * 1.e7), self.get_minor_0(self.lat) * 1.e7, float(self.lon * 1.e7), self.get_minor_0(self.lon) * 1.e7)
        #
        # CRITICAL
        data_to_send = data_to_send + data_to_send_3
        #
        if is_debugging:
            print("[fsm] data_to_send 3")
            print(data_to_send_3)
            unpack_format_t_3 = str()
            for i in range(0, self.swarm_num):
                unpack_format_t_3 = unpack_format_t_3 + "B"
            unpack_format_t_3 = unpack_format_t_3 + "fff"
            unpack_format_t_3 = unpack_format_t_3 + "ffff"            
            unpacked_t = struct.unpack(unpack_format_t_3, data_to_send_3)
            print(unpacked_t)
            #
            adjacency_mat_debug = np.zeros([self.swarm_num, self.swarm_num])
            for i in range(0, self.swarm_num):
                mat_this_line = unpacked_t[i]
                for j in range(0, self.swarm_num):
                    if mat_this_line & 0x01 > 0:
                        adjacency_mat_debug[i, j] = 1
                    mat_this_line = (mat_this_line >> 1) & 0x00FF
            print(adjacency_mat_debug)
            #
            print("[fsm] data_to_send 1 + 2 + 3")
            unpack_format_t_2 = "fff"
            for i in range(0, self.swarm_num):
                unpack_format_t_2 = unpack_format_t_2 + "f"
            #
            print(data_to_send)
            print(struct.unpack("BBBIBB", data_to_send[0:10]))
            print(struct.unpack(unpack_format_t_2, data_to_send[10:54]))
            print(struct.unpack(unpack_format_t_3, data_to_send[54:]))
            print(len(data_to_send))

        #
        data_to_send_4 = bytes()
        if is_debugging:
            self.tgt_list   = [[2, 100.5, -125.1], [1, 50, 150], [-1, 0, 0]]
            self.guard_list = [[2, 3], [1, 7], [-1, 0]]
        # self.tgt_list                         # (int8_t + float32 + float32) * 3, in meters
        # self.guard_list                       # (int8_t + int8_t) * 3, in meters
        # self.current_tgt
        # self.mission_status
        #
        for i in range(0, self.known_target_num):
            data_to_send_4 = data_to_send_4 + struct.pack("=bff", self.tgt_list[i][0], self.tgt_list[i][1], self.tgt_list[i][2])
        #
        for i in range(0, self.known_target_num):
            data_to_send_4 = data_to_send_4 + struct.pack("=bb", self.guard_list[i][0], self.guard_list[i][1])
        data_to_send_4 = data_to_send_4 + struct.pack("=bb", self.current_tgt, self.mission_status)
        #
        # CRITICAL
        data_to_send = data_to_send + data_to_send_4
        #
        if is_debugging:
            print("[fsm] data_to_send 4")
            print(data_to_send_4)
            #
            unpack_format_t_4 = str()
            for i in range(0, self.known_target_num):
                unpack_format_t_4 = unpack_format_t_4 + "bff"
            for i in range(0, self.known_target_num):
                unpack_format_t_4 = unpack_format_t_4 + "bb"
            unpack_format_t_4 = unpack_format_t_4 + 'bb'
            print(struct.unpack(unpack_format_t_4, data_to_send_4))
            #
            #
            print("[fsm] data_to_send 1 + 2 + 3 + 4")
            #
            unpack_format_t_1 = "BBBIBB"
            #
            unpack_format_t_2 = "fff"
            for i in range(0, self.swarm_num):
                unpack_format_t_2 = unpack_format_t_2 + "f"
            #
            unpack_format_t_3 = str()
            for i in range(0, self.swarm_num):
                unpack_format_t_3 = unpack_format_t_3 + "B"
            unpack_format_t_3 = unpack_format_t_3 + "fff"
            unpack_format_t_3 = unpack_format_t_3 + "ffff"
            #
            print(struct.unpack(unpack_format_t_1, data_to_send[0:10]))
            print(struct.unpack(unpack_format_t_2, data_to_send[10:54]))
            print(struct.unpack(unpack_format_t_3, data_to_send[54:90]))
            print(struct.unpack(unpack_format_t_4, data_to_send[90:]))
        print(f"[fsm] data_to_send len: {len(data_to_send)}")
        return data_to_send

    def Unpack_Info(self, data_received, is_debugging=False):
        if data_received == None:
            return []

        data_t = {}
        # print(f"[fsm] got data len: {len(data_received)}")
        #
        unpack_format_t_1 = "=BBBIB"
        unpacked_1 = struct.unpack(unpack_format_t_1, data_received[0:8])
        if is_debugging:
            print("[fsm] unpacking common data: ")
            print(unpacked_1)
        #
        if unpacked_1[0] != 0xAB or unpacked_1[1] != 0xEF:
            print("[fsm] 数据校验失败, 帧头不对 ...")
            return []

        #
        # 公共信息
        data_t['frame_id'] = unpacked_1[2]
        data_t['time']     = unpacked_1[3]
        data_t['uav_id']   = unpacked_1[4]

        if data_t['frame_id'] == 1:
            #
            unpack_1_end = 8
            #
            unpack_format_t_2 = "=Bfff"
            unpack_2_end = unpack_1_end + 13
            for i in range(0, self.swarm_num):
                unpack_format_t_2 = unpack_format_t_2 + "f"
                unpack_2_end = unpack_2_end + 4
            #
            unpack_format_t_3 = "="
            unpack_3_end = unpack_2_end
            for i in range(0, self.swarm_num):
                unpack_format_t_3 = unpack_format_t_3 + "B"
                unpack_3_end = unpack_3_end + 1
            unpack_format_t_3 = unpack_format_t_3 + "fff"
            unpack_format_t_3 = unpack_format_t_3 + "ffff"
            unpack_3_end = unpack_3_end + 7 * 4
            #
            unpack_format_t_4 = "="
            for i in range(0, self.known_target_num):
                unpack_format_t_4 = unpack_format_t_4 + "bff"
            for i in range(0, self.known_target_num):
                unpack_format_t_4 = unpack_format_t_4 + "bb"
            unpack_format_t_4 = unpack_format_t_4 + 'bb'
            
            unpacked_2 = struct.unpack(unpack_format_t_2, data_received[unpack_1_end:unpack_2_end])  # 这个6是试出来的, 不要问我咋知道的, TODO, check correctness! 如果错了则改回unpack_format_t_1 = "BBBIBB", unpack_format_t_2 = "fff"
            unpacked_3 = struct.unpack(unpack_format_t_3, data_received[unpack_2_end:unpack_3_end])
            unpacked_4 = struct.unpack(unpack_format_t_4, data_received[unpack_3_end:])
            
            #
            data_t['group_id']  = unpacked_2[0]
            data_t['vel']       = [ unpacked_2[1], unpacked_2[2], unpacked_2[3] ]
            
            # print(f"[fsm] {[i for i in range(0, self.swarm_num)]}")
            data_t['dist_list'] = [ unpacked_2[4 + i] for i in range(0, self.swarm_num) ]
            #
            adjacency_mat_debug = np.zeros([self.swarm_num, self.swarm_num])
            for i in range(0, self.swarm_num):
                mat_this_line = unpacked_3[i]
                for j in range(0, self.swarm_num):
                    if mat_this_line & 0x01 > 0:
                        adjacency_mat_debug[i, j] = 1
                    mat_this_line = (mat_this_line >> 1) & 0x00FF
            #
            data_t['pos']  = [ unpacked_3[self.swarm_num], unpacked_3[self.swarm_num + 1], unpacked_3[self.swarm_num + 2] ]
            data_t['lat']  = self.my_floor(unpacked_3[self.swarm_num + 3] / 1.e7) + unpacked_3[self.swarm_num + 4] / 1.e7
            data_t['lon']  = self.my_floor(unpacked_3[self.swarm_num + 5] / 1.e7) + unpacked_3[self.swarm_num + 6] / 1.e7
            #
            recv_tgt_list_len = self.known_target_num * 3
            data_t['tgt_list'] = []
            for i in range(0, self.known_target_num):
                tgt_list_t = [unpacked_4[i * 3], unpacked_4[i * 3 + 1], unpacked_4[i * 3 + 2]]
                data_t['tgt_list'].append(tgt_list_t)
            #
            recv_guard_list_len = self.known_target_num * 2
            data_t['guard_list'] = []
            for i in range(0, self.known_target_num):
                tgt_list_t = [unpacked_4[i * 2 + recv_tgt_list_len], unpacked_4[i * 2 + 1 + recv_tgt_list_len]]
                data_t['guard_list'].append(tgt_list_t)
            data_t['current_tgt']  = unpacked_4[recv_tgt_list_len + recv_guard_list_len]
            data_t['mission_stat'] = unpacked_4[recv_tgt_list_len + recv_guard_list_len + 1]

            if is_debugging:
                print("[fsm] received data, id = 1: ")
                print(unpacked_1)
                print(unpacked_2)
                print(unpacked_3)
                print(unpacked_4)
                #
                print(data_t)

        elif data_t['frame_id'] == 2:
            unpacked_t = struct.unpack("BBBIBI",   data_received)
            data_t['cmd_id'] = unpacked_t[5]

            if is_debugging:
                print(unpacked_t)
                print(data_t)
        elif data_t['frame_id'] == 3:
            unpacked_t = struct.unpack("BBBIBIBB", data_received)
            data_t['cmd_id'] = unpacked_t[5]
            data_t['companion_id'] = unpacked_t[6]
            data_t['ack'] = unpacked_t[7]

            if is_debugging:
                print(unpacked_t)
                print(data_t)

        return data_t

    def Pack_Critical_Status(self, cmd_id, is_debugging=False):
        # 帧头    帧头    数据帧ID  时间    UAV_ID    cmd_id
        # 0xAB   0xEF      2       ?       ?         ?
        self.header1 = 0xAB                           # uint8_t
        self.header2 = 0xEF                           # uint8_t
        frame_id = 2                                  # uint8_t
        self.time_t = time.time() - self.t_start      # uint32_t *1000 -> ms
        self.id                                       # uint8_t
        cmd_id                                        # uint16_t
        data_to_send = struct.pack("=BBBIBI", self.header1, self.header2, frame_id, int(self.time_t * 1000), self.id, cmd_id)

        return data_to_send
  
    def Pack_Critical_Status_Ack(self, companion_id, cmd_id, is_debugging=False):
        # 帧头    帧头    数据帧ID  时间  UAV_ID    cmd_id    companion_uav_id  ACK
        # 0xAB   0xEF      3       ?     ?         ?              ?           1
        self.header1 = 0xAB                           # uint8_t
        self.header2 = 0xEF                           # uint8_t
        frame_id = 3                                  # uint8_t
        self.time_t = time.time() - self.t_start      # uint32_t *1000 -> ms
        self.id                                       # uint8_t
        cmd_id                                        # uint16_t
        companion_id                                  # uint8_t
        fsm_command.command_ack
        data_to_send = struct.pack("=BBBIBIBB", self.header1, self.header2, frame_id, int(self.time_t * 1000), self.id, cmd_id, companion_id, fsm_command.command_ack)

        return data_to_send

    def ReceiveInfo(self,send_uav_id,info_type):
        while True:
            if self.udpc.data:
                if info_type==1:
                    #if the tag info haven't been stored, store it
                    pass
                    # if tag_id == self.group:
                        # make a bind with uav in same group to decide which uav should be a station
                elif info_type==2:
                    #if the bind signal is received,
                    pass
                elif info_type==3:
                    #if the trajectory have conflict with self, according to the priority, change the trajectory
                    pass
                elif info_type==4:
                    pass
                    self.udpc.data=[]
      
  
    def UWBProcess(self):
        '''
        according to the uwb data, estimate the pos 
        '''
        
        #a matrix store the estimation of uwb data
        useful_pos_list=[]
        useful_uwb_estimation=[]
        for i in range(self.swarm_num):
            if self.swarm_states_list[i]==1:
                station_pos=self.swarm_pos_list[i]
                if(station_pos.norm()>0):
                    useful_pos_list.append(station_pos)
                    useful_uwb_estimation.append(self.uwb_estimation[i])
        # estimate pos by triangulation
  
  
    def DetectorProcess(self):
        '''
        detect the tag
        receive the detection result from another node
        need to estimate the pos of the tag(beneficial for info sharing)
        '''
        
        #tag_id=
        #tag_pos=
        # self.tag_list[tag_id]=tag_pos
        self.fsm_state='tag'
        
        self.SendInfo(self,1,1)
          
    def UAVControl(self):#timer
        '''
        send control sig to fcu 
        since control is simple, the planned path is our traj, and the time is cal by the dis
        '''
        pass
    
    def GimbalControl(self):#timer
        '''
        send control sig to gimbal
        '''
        pass

    def sort_tgt_list_guard_list(self, x, y):
        '''
            usage:
            strs.sort(key=functools.cmp_to_key(my_compare), reverse=True)
        '''
        if x[0] > y[0]:
            return 1
        elif x[0] < y[0]:
            return -1
        return 0
    
    def get_minor_0(self, x):
        return float(x - self.my_floor(x))  # 取小数

    def my_floor(self, x):
        if x > 0.:
            return math.floor(x)
        else:
            return math.ceil(x)             # e.g. floor(-118.1) = -119, 所以要这个函数, 不然不好取小数位

'''

'''
# fsm_t = fsm(uav_id, swarm_num, known_target_num, uav_group_id, target_port=target_port)

# if __name__ == "__main__":

#     try:
#         schedule.every(1).seconds.do(fsm_t.FSMCallback)

#         fsm_t.udpc.start_receive_thread()

#         while True:  
#             schedule.run_pending()   # 运行待执行的任务队列
#             time.sleep(0.01)         # 这个刷新速度要小于scheduled task的
#     except KeyboardInterrupt:
#         print("[fsm] 服务器正在关闭...")
#     finally:
#         fsm_t.shutdown()

#     print("[fsm] all stopped ...")