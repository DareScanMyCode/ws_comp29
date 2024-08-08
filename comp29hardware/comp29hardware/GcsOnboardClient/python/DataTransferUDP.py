# ffmpeg -re -i output.avi -c copy -f rtsp rtsp://localhost:8554/mystream

import socket
import threading
import time
from PIL import Image
import io
import cv2
from math import ceil
import zlib
import numpy as np
class DataTransferUDP:
    """
    Data frame
        Bytes: 0-3: Image num (int) (Data[0] == 0xff --> Data End Frame)
        Bytes: 4-7: Image width (int)
        Bytes: 8-11: Image height (int)
        Bytes: 12-15: Image frame now (int)
        Bytes: 16-19: total frame (int)
        Bytes: 20-: Image data (bytes)
    Res frame
        Bytes:  0:      0x00 for recv success
                        0xff for frame lost
        Bytes:  1:end   lost frame id
    Data End Frame
        Bytes:  0xff    Data end
    """
    MAX_DATA_SIZE = 1024
    def __init__(self, local_ip, remote_ip, local_recv_port, remote_recv_port, mode="sender"):
        self.local_ip = local_ip
        self.remote_ip = remote_ip
        self.local_recv_port = local_recv_port
        self.remote_recv_port = remote_recv_port
        self.mode = mode
        
        self.image = None
        self.last_recv_time = None
        self.img_info = {
            'img_num': 0,
            'img_width': 0,
            'img_height': 0
        }
        self.img_num = -1
        self.last_img_num = -1
        self.recv_frame_now = 0
        self.recv_correct_flag = True
        self.current_img = None
        self.recv_bytes = b""
        self.recv_frame_num_current = 0
        self.sending_num = 0
        self.frame_got = []
        self.img_total_frame = 0
        self.img_to_send = None
        print(f"[INFO] ImageClient: {mode} mode")
        self.should_stop = False
        self.compressed_data = None
        self.receiver_recved = False
        self.receiver_ready = True
        self.data_recved = 0
        try:
            if mode == "sender":
                self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sender_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sender_receiver.bind((self.local_ip, self.local_recv_port))
                self.sender_receiver.settimeout(0.3)
                # print(f"Sender receiver binded to {self.local_ip}:{self.local_recv_port}")
                t = threading.Thread(target=self.sender_recv_t)
                t.start()
                # print("Client connected")
            elif mode == "receiver":
                self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.receiver.bind((self.local_ip, self.local_recv_port))
                self.receiver_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # print(f"Receiver sender binded to {self.local_ip}:{self.local_recv_port}")
                self.receiver.settimeout(0.3)
                t = threading.Thread(target=self.receiver_recv_t)
                t.start()
            # print("Server started")
        except OSError:
            print("Error creating socket")
            pass
            
    def get_current_img(self):
        return self.current_img
    
    def send(self, img):
        # print("Sending image")
        # print(type(img))
        img_size = img.shape
        img_height = img_size[0]
        img_width = img_size[1]
        compressed_data = zlib.compress(img.tobytes(), level=9)
        frame_now = 0
        total_frame = ceil(img_height * img_width * 3 / self.MAX_DATA_SIZE)
        # print(f"Image size: {img_size}")
        # print(f"Total frame: {total_frame}")
        # input("Press enter to continue")
        img = img.reshape(-1)  # flatten image
        for frame in range(total_frame):
            img: np.ndarray
            frame_now = frame + 1
            index_begin = frame * self.MAX_DATA_SIZE
            index_end = (frame + 1) * self.MAX_DATA_SIZE
            index_end = index_end if index_end < len(img) else len(img)
            img_data = img[frame * self.MAX_DATA_SIZE: (frame + 1) * self.MAX_DATA_SIZE]
            # img_data = img.crop((frame * self.MAX_DATA_SIZE, 0, (frame + 1) * self.MAX_DATA_SIZE, img_height))
            img_data = img_data.tobytes()
            data =  self.sending_num.to_bytes(4, byteorder='big') +\
                    img_width.to_bytes(4, byteorder='big') +\
                    img_height.to_bytes(4, byteorder='big') +\
                    frame_now.to_bytes(4, byteorder='big') +\
                    total_frame.to_bytes(4, byteorder='big') +\
                    img_data
            self.sender.sendto(data, (self.remote_ip, self.remote_recv_port))
            # print(f"Sending frame {frame_now}/{total_frame}, frame size: {len(data)}")
            # time.sleep(1e-12)
    
    def gen_data_frame(self, data, frame_now, total_frame, img_height, img_width, data_num):
        index_begin = (frame_now - 1)* self.MAX_DATA_SIZE
        index_end = frame_now * self.MAX_DATA_SIZE
        index_end = index_end if index_end < len(data) else len(data)
        frame_data = data[index_begin: index_end]
        payload =   data_num.to_bytes(4, byteorder='big') +\
                    img_width.to_bytes(4, byteorder='big') +\
                    img_height.to_bytes(4, byteorder='big') +\
                    frame_now.to_bytes(4, byteorder='big') +\
                    total_frame.to_bytes(4, byteorder='big') +\
                    frame_data
        return payload
    
    def send_compresses_img(self, img):
        if not self.receiver_ready:
            # print("Receiver not ready")
            return False
        self.sending_num += 1
        self.receiver_ready = False
        self.img_to_send = img
        img_size = img.shape
        img_height = img_size[0]
        img_width = img_size[1]
        is_success, buffer = cv2.imencode(".jpg", img)
        self.compressed_data = zlib.compress(buffer.tobytes())
        frame_now = 0
        total_frame = ceil(len(self.compressed_data) / self.MAX_DATA_SIZE)
        img = img.reshape(-1)  # flatten image
        for frame in range(total_frame):
            frame_now = frame + 1
            payload = self.gen_data_frame(self.compressed_data, frame_now, total_frame, img_height, img_width, self.sending_num)
            self.sender.sendto(payload, (self.remote_ip, self.remote_recv_port))
            t = time.time()
            # while time.time() - t < 0.001:
            #     pass
        self.sender.sendto(self.gen_data_end_msg(), (self.remote_ip, self.remote_recv_port))
        
        # print(f"Image {self.sending_num} sent")
        return True
    
    def sender_recv_t(self):
        while not self.should_stop:
            try:
                self.sender_recv()
            except socket.timeout:
                pass
                # print("Timeout")
    def gen_data_end_msg(self):
        return b"\xff" + self.sending_num.to_bytes(4, byteorder='big') + len(self.compressed_data).to_bytes(4, byteorder='big')
    
    def sender_recv(self):
        data, addr = self.sender_receiver.recvfrom(self.MAX_DATA_SIZE+30)
        if data[0] == 0x00:
            # ("No loss frame")
            self.receiver_ready = True
            return
        if data[0] == 0xff:
            loss_frame = [int.from_bytes(data[i:i+4], byteorder='big') for i in range(1, len(data), 4)]
            # (f"Loss frames: {loss_frame}")
        for frame in loss_frame:
            payload = self.gen_data_frame(self.compressed_data, frame + 1, self.img_total_frame, self.img_info['img_height'], self.img_info['img_width'], self.sending_num)
            self.sender.sendto(payload, (self.remote_ip, self.remote_recv_port))
        self.sender.sendto(self.gen_data_end_msg(), (self.remote_ip, self.remote_recv_port))
        
    def receiver_recv_t(self):
        while not self.should_stop:
            try:
                self.receiver_recv()
            except socket.timeout:
                # print("Timeout, Receiver ready")
                self.send_ready()

            # except Exception as e:
            #     (e)
            #     ("Error")
            #     break
    def frame_check(self, total_num = -1):
        if total_num == -1:
            total_num = self.img_total_frame
        loss_frame = []
        for frame_id in range(total_num):
            if frame_id + 1 not in self.frame_got:
                loss_frame.append(frame_id)
        if len(loss_frame) != 0:
            print(f"Loss frame: {loss_frame}")
            return False, loss_frame
        else:
            # ("No loss frame")
            return True, loss_frame
    def send_ready(self):
        feedback = b"\x00"
        self.receiver_sender.sendto(feedback, (self.remote_ip, self.remote_recv_port))
        
    def receiver_recv(self):
        data, addr = self.receiver.recvfrom(self.MAX_DATA_SIZE+30)
        if data[0] == 0xff:
            data_num = int.from_bytes(data[1:5], byteorder='big')
            data_len = int.from_bytes(data[5:9], byteorder='big')
            # frame ended
            # TODO 如果收不到呢
            # TODO 校验
            check_flag, loss_frame = self.frame_check()
            if check_flag:
                # np.reshape(self.recv_array, -1)
                self.recv_array = self.recv_array[:data_len]
                # ("Frame check passed")
                # (f"len{len(self.recv_array)}")
                try:
                    decompressed_data = zlib.decompress(self.recv_array)
                    # 将字节流转换回图像
                    self.current_img = cv2.imdecode(np.frombuffer(decompressed_data, np.uint8), cv2.IMREAD_COLOR)
                    self.send_ready()
                    self.data_recved += 1
                    cv2.imshow('image', self.current_img)
                    cv2.waitKey(1)
                except zlib.error:
                    print("Decompress error")
                    return False
                
                # cv2.destroyAllWindows()
            else:
                feedback = b"\xff" + b"".join([i.to_bytes(4, byteorder='big') for i in loss_frame])  # \xff表示有丢帧
                print("Frame check failed")
                self.receiver_sender.sendto(feedback, (self.remote_ip, self.remote_recv_port))
                return
        self.img_num = int.from_bytes(data[0:4], byteorder='big')
        img_width = int.from_bytes(data[4:8], byteorder='big')
        img_height = int.from_bytes(data[8:12], byteorder='big')
        img_frame_now = int.from_bytes(data[12:16], byteorder='big')
        self.img_total_frame = int.from_bytes(data[16:20], byteorder='big')
        img_data = data[20:]
        if self.img_num != self.last_img_num:
            self.last_img_num = self.img_num
            self.img_info['img_num'] = self.img_num
            self.img_info['img_width'] = img_width
            self.img_info['img_height'] = img_height
            self.recv_frame_now = 0
            self.recv_bytes = b""
            self.frame_got = []
            self.recv_array = bytearray(self.img_total_frame * self.MAX_DATA_SIZE)
        self.recv_frame_num_current += 1
        if self.recv_frame_num_current != img_frame_now:
            self.recv_correct_flag = False
        if self.recv_correct_flag:
            self.recv_bytes += img_data
            self.recv_array[(img_frame_now-1)*self.MAX_DATA_SIZE: (img_frame_now-1)*self.MAX_DATA_SIZE+len(img_data)] = img_data
        else:
            self.recv_array[(img_frame_now-1)*self.MAX_DATA_SIZE: (img_frame_now-1)*self.MAX_DATA_SIZE+len(img_data)] = img_data
        self.frame_got.append(img_frame_now)
        if img_frame_now % 5000 == 0:
            # 每50帧检查一次
            check_flag, loss_frame = self.frame_check(img_frame_now - 1)
            if not check_flag:
                feedback = b"\xff" + b"".join([i.to_bytes(4, byteorder='big') for i in loss_frame])
                self.receiver_sender.sendto(feedback, (self.remote_ip, self.remote_recv_port))
        # if img_frame_now % 88 == 0:
        #     (f"Receiving frame {self.recv_frame_num_current}/{img_frame_now}/{self.img_total_frame}, img_info:w {img_width} h {img_height} num {self.img_num}")

    def close(self):
        self.sender.close() if self.mode == "client" else self.receiver.close()
        print("Closed")
        

def test_one_frame():
    receiver = DataTransferUDP("127.0.0.1", 30000, mode="receiver")
    sender = DataTransferUDP("127.0.0.1", 30000, mode="sender")
    try:
        # img = Image.open("./test.jpg")
        img = cv2.imread("./test.jpg")
        # cv2.imshow('image', img)
        # cv2.waitKey(0)
        sender.send_compresses_img(img)
        # client.close()
        print("Done")
        print(sender.recv_frame_num_current)
    except KeyboardInterrupt:
        receiver.should_stop = True
        sender.should_stop = True
        print(receiver.recv_frame_num_current)
    while True:
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    # server.close()
    receiver.should_stop = True
    print(receiver.should_stop)
    # except:
    #     print("Error sending image")
    #     pass
    
def test_video():
    from tqdm import tqdm
    receiver = DataTransferUDP("127.0.0.1", 30000, mode="receiver")
    sender = DataTransferUDP("127.0.0.1", 30000, mode="sender")
    video = cv2.VideoCapture(0)
    try:
        ret, img = video.read()
        
        # progress_bar = tqdm(total=100)
        while ret:
            img = cv2.resize(img, (640, 480))
            re = sender.send_compresses_img(img)
            time.sleep(0.05)
            print("\r"*30 + f"Recv: {receiver.data_recved} / {sender.sending_num}", end="")
            ret, img = video.read()
        #     progress_bar.update(1)
        # progress_bar.close()
        print("Done")
    except KeyboardInterrupt:
        receiver.should_stop = True
        sender.should_stop = True
        time.sleep(3)
        print("Server stopped")
        exit()
def test_receiver():
    receiver = DataTransferUDP("127.0.0.1", "127.0.0.1", 30000, 30001, mode="receiver")
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            receiver.should_stop = True
            break

def test_sender():
    sender = DataTransferUDP("127.0.0.1", "127.0.0.1", 30001, 30000, mode="sender")
    video = cv2.VideoCapture(0)
    try:
        ret, img = video.read()
        while ret:
            img = cv2.resize(img, (640, 480))
            re = sender.send_compresses_img(img)
            print(re)
            time.sleep(0.05)
            ret, img = video.read()
        print("Done")
    except KeyboardInterrupt:
        sender.should_stop = True
        time.sleep(3)
        print("Server stopped")
        exit()

class DataTransTCPSender:
    def __init__(self, target_ip, target_port) -> None:
        self.target_ip = target_ip
        self.target_port = target_port
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sender.connect((target_ip, target_port))
        pass
    
    def send_img(self, img):
        is_success, buffer = cv2.imencode(".jpg", img)
        compressed_data = zlib.compress(buffer.tobytes())
        header = b'\x00\x00\x00\x00'
        header += len(compressed_data).to_bytes(4, byteorder='big')
        self.sender.sendall(header)
        self.sender.sendall(compressed_data)
        
class DataTransTCPReceiver:
    def __init__(self, local_ip, local_port) -> None:
        self.local_ip = local_ip
        self.local_port = local_port
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.current_img = None
        self.receiver.bind((local_ip, local_port))
        t = threading.Thread(target=self.recv_img_t)
        t.start()
        
        pass
    def get_current_img(self):
        return self.current_img
    
    
    def recv_img_t(self):
        self.receiver.listen(1)
        self.conn, self.addr = self.receiver.accept()
        while True:
            data = self.conn.recv(1024)
            if len(data) == 8:
                header = data[:4]
                if header == b'\x00\x00\x00\x00':
                    data_len = int.from_bytes(data[4:], byteorder='big')
                    print(f"Data length: {data_len}")
                    self.current_img = self.recv_img(data_len)
                print("Received")
                cv2.imshow('image', self.current_img)
                cv2.waitKey(1)
            # print(data)
    
    def recv_img(self, data_len):
        data = b''
        while len(data) < data_len:
            data += self.conn.recv(data_len-len(data))
        decompressed_data = zlib.decompress(data)
        # 将字节流转换回图像
        decom_img = cv2.imdecode(np.frombuffer(decompressed_data, np.uint8), cv2.IMREAD_COLOR)
        return decom_img
    
def test_TCP_send():
    sender = DataTransTCPSender("127.0.0.1", 30000)
    video = cv2.VideoCapture(0)
    try:
        ret, img = video.read()
        while ret:
            img = cv2.resize(img, (640, 480))
            re = sender.send_img(img)
            # print(re)
            time.sleep(0.05)
            ret, img = video.read()
        print("Done")
    except KeyboardInterrupt:
        sender.should_stop = True
        time.sleep(3)
        print("Server stopped")
        exit()

def test_TCP_recv():
    receiver = DataTransTCPReceiver("127.0.0.1", 30000)

if __name__ == "__main__":
    ### TCP test
    # t1 = threading.Thread(target=test_TCP_recv)
    # t1.start()
    # t2 = threading.Thread(target=test_TCP_send)
    # t2.start()
    
    ### UDP test
    t1 = threading.Thread(target=test_receiver)
    t2 = threading.Thread(target=test_sender)
    t1.start()
    t2.start()
    
    # while True:
    #     try:
    #         print("Main loop")
    #         time.sleep(1)
    #     except KeyboardInterrupt:
    #         break
    