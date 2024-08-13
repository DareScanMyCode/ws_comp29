#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import selectors
import struct
import threading
import time
import zlib
from threading import Thread

class UDPCommunicator:
    def __init__(self, port=25001):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))              # IP一般不写
        #self.sock.setblocking(False)
        #self.selector = selectors.DefaultSelector()
        #self.selector.register(self.sock, selectors.EVENT_READ)
        self.peers = {}  # 用于存储已知对端的信息
        
        self.data=[]

        #
        # [
        #  [data_1, addr_1],
        #  [data_2, addr_2],
        #  ...
        # ]
        self.recv_data_list = []
        self.is_debugging = False
    '''
    def send(self, target_ip,target_port, message, frame_format='>I4s4s', header_size=8):
        # message is a list
        header = struct.pack(frame_format, len(message), 0)  # 自定义帧头
        message_bytes = header + message.encode('utf-8')
        self.sock.sendto(message_bytes, (target_ip, target_port))
    '''

    def send(self, message, target_ip, target_port):
        try:
            self.sock.sendto(message, (target_ip, target_port))
            return True
        except:
            # print(f"[udp process] send error {target_ip}@{target_port}")
            return False
            
        # return self.sock.sendto(message, (target_ip, target_port))

    def start_receive_thread(self):
        self.recv_thread_t = threading.Thread(target=self.receive)
        self.recv_thread_t.start()

    '''
    def receive(self):
      while True:
        events = self.selector.select(timeout=0)  # 设置超时时间为0，即非阻塞模式
        for key, mask in events:
          if key.fileobj is self.sock:
            data, addr = self.sock.recvfrom(1024)
            self.parse_message(data, addr)
            # self.data=
    '''
    def receive(self):
        print("[udp process] receive thread started ...")
        while True:
            try:
                #events = self.selector.select(timeout=0.1)
                data_t, addr_t = self.sock.recvfrom(1024)
                self.recv_data_list.append([data_t, addr_t])
                #
                if self.is_debugging:
                    print("[udp process] received: from %s, %s" % (str(addr_t), str(data_t), ))
                #
                time.sleep(0.005)                                                   # TODO
            except socket.timeout:
                print("[udp process] 连接超时，请检查网络或目标主机是否可达")
            except BlockingIOError:
                print("[udp process] WARING: 无法立即完成一个非阻止性套接字操作")

    def pack_message(self,msg_type,message):
       '''
       id: 1 byte 说明信息类型 1: tag 2:b bind 3: trajectory 4: heartbeat
       length: 4 byte
       data:
       '''
       if msg_type==1:
         '''
         tag_id: 1 byte
         tag_time: 4 byte
         tag_pos: 8 byte(2d)
         '''
         tag_id=message[0]
         tag_time=message[1]
         tag_pos_x=message[2]
         tag_pos_y=message[3]
         tag_pos_z=message[4]
         message = struct.pack('>BIddd', tag_id, tag_time, tag_pos_x, tag_pos_y, tag_pos_z)
       elif msg_type==2:
         '''
         uav_id: 1 byte
         bind_num: 4 byte
         '''
         uav_id=message[0]
         bind_num=message[1]
         message=struct.pack('>BI',uav_id, bind_num)
       elif msg_type==3:
        #  length=len(message)
        #  message=struct.pack('>I',length)+message
        pass
       elif msg_type==4:
        '''
        uav_id: 1 byte
        heartbeat: 1 byte
        '''
        uav_id=message[0]
        heartbeat=message[1]
        message=struct.pack('>BI',uav_id,heartbeat)

    def parse_message(self, data):
        info_type=struct.unpack('>B',data[0:1])
        length=struct.unpack('>I',data[1:5])
        message=data[5:]
        if info_type==1:
            #if the tag info haven't been stored, store it
            pass
            # if tag_id == self.group:
            # make a bind with uav in same group to decide which uav should be a station
            # TODO
        elif info_type==2:
            #if the bind signal is received,
            # TODO
            pass
        elif info_type==3:
            #if the trajectory have conflict with self, according to the priority, change the trajectory
            # TODO
            pass
        elif info_type==4:
            # TODO
            pass

        #return a dict
        # TODO
        data_to_send = []
        return data_to_send

    def close(self):
        #self.selector.unregister(self.sock)
        self.sock.close()
        #self.selector.close()

# communicator = UDPCommunicator(local_port=12345)

# def udp_send_thread():
#    pass

# if __name__ == "__main__":

#     send_thread_t = Thread(target=udp_send_thread, name="udp_send_thread")
#     send_thread_t.start()

#     try:
#         print("[communicator] UDP服务器启动, 等待接收消息...")
#         communicator.receive()
#     except KeyboardInterrupt:
#         print("[communicator] 服务器正在关闭...")
#     finally:
#         communicator.close()

#     send_thread_t.join()

#     print("[communicator] all stopped ...")
