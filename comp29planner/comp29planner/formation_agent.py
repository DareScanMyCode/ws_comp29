import numpy as np
import math


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
    
    

class Leader:
    def __init__(self, index_id, adj_mtx, tgt_pos) -> None:
        """
        纯正的leader，负责带动整个队形运动。需要把自身的速度发送给别的无人机
        """
        self.index_id = index_id
        self.adj_mtx = adj_mtx
        self.tgt_pos = tgt_pos
        pass
    
    def get_ctrl(self, dist=None, dx=None):
        pass
    
    def update_tgt_pos(self, new_pos):
        return
        # self.tgt_pos = new_pos if isinstance(new_pos, np.ndarray) else np.array(new_pos)
        # self.tgt_pos_local = self.tgt_pos-self.tgt_pos[self.index_id]
        # self.tgt_dist = np.linalg.norm(self.tgt_pos_local, axis=1)
        # self.dist_only_agent.update_tgt_pos(new_pos=new_pos)
    
    def update_adj(self, aaa):
        return

    
class AngleLeader:
    def __init__(self, index_id, adj_mtx, tgt_pos, leader_index_id, k_dist=0.15, ki_dist=0.003, k_img=0.001, ki_img=0.00001) -> None:
        """
        第二leader，负责带动整个队形运动，控制自身与leader之间的角度。
        """
        self.index_id = index_id
        self.adj_mtx = adj_mtx if isinstance(adj_mtx, np.ndarray) else np.array(adj_mtx)
        self.tgt_pos = tgt_pos if isinstance(tgt_pos, np.ndarray) else np.array(tgt_pos)
        self.tgt_pos_local = self.tgt_pos-self.tgt_pos[self.index_id]
        self.tgt_dist = np.linalg.norm(self.tgt_pos_local, axis=1)
        # print(self.tgt_dist)
        self.leader_index_id = leader_index_id
        # print(self.leader_index_id)
        self.k_dist = k_dist
        self.k_img  = k_img
        self.dist_err_int = 0.
        self.img_err_int = 0.
        self.ki_dist = ki_dist
        self.ki_img = ki_img
        self.dist_only_agent = Follower(self.index_id, self.adj_mtx, self.tgt_pos, self.k_dist,self.ki_dist)
        pass
    
    def update_adj(self, new_adj):
        self.adj_mtx = new_adj
        self.dist_only_agent.update_adj(new_adj=new_adj)
        
    def update_tgt_pos(self, new_pos):
        self.tgt_pos = new_pos if isinstance(new_pos, np.ndarray) else np.array(new_pos)
        self.tgt_pos_local = self.tgt_pos-self.tgt_pos[self.index_id]
        self.tgt_dist = np.linalg.norm(self.tgt_pos_local, axis=1)
        self.dist_only_agent.update_tgt_pos(new_pos=new_pos)
        
    
    def get_ctrl(self, dist, dx=None):
        vv_frd = [0.0, 0.0]
        
        if dx is not None:
            # 有dx，有观测到leader的角度，只控制与leader的相对位置
            leader_dist_err = self.tgt_dist[self.leader_index_id] - dist[self.leader_index_id]
            self.dist_err_int += leader_dist_err
            self.dist_err_int = 100 if self.dist_err_int > 100 else self.dist_err_int
            self.dist_err_int = -100 if self.dist_err_int < -100 else self.dist_err_int
            # 此处假设要让leader在自己的正前方，即dist控制前后，dx控制左右，
            # dist err 为正，距离过近，速度向后
            vf = - (self.k_dist * leader_dist_err + self.ki_dist * self.dist_err_int)
            # print(f"[1111] dist_err={leader_dist_err}")
            # dx为正，leader在左边！（dx为期望像素坐标-img中leader的像素坐标），左上角为零，向左
            self.img_err_int += dx
            self.img_err_int = 10000 if self.img_err_int > 10000 else self.img_err_int
            self.img_err_int = -10000 if self.img_err_int < -10000 else self.img_err_int
            if math.fabs(dx) < 30:
                vr = 0.0
            else:
                vr = - self.k_img * dx
            vv_frd = [vf, vr]
        else:
            # 无dx，说明没检测到leader，只进行distance only的编队
            vv_frd = self.dist_only_agent.get_ctrl(dist=dist)
        return vv_frd
    
class Follower:
    def __init__(self, index_id, adj_mtx, tgt_pos, k_dist=0.1, ki_dist=0.001) -> None:
        """
        纯正的follower，观测，运动
        """
        self.index_id = index_id
        self.adj_mtx = adj_mtx if isinstance(adj_mtx, np.ndarray) else np.array(adj_mtx)
        self.tgt_pos = tgt_pos if isinstance(tgt_pos, np.ndarray) else np.array(tgt_pos)
        self.tgt_pos_local = self.tgt_pos-self.tgt_pos[self.index_id]
        self.tgt_dist = np.linalg.norm(self.tgt_pos_local, axis=1)
        # print(self.tgt_pos_local)
        # print(self.tgt_dist)
        self.k_dist = k_dist
        self.ki_dist = ki_dist
        self.error_int = None
        
        pass
    
    def update_adj(self, new_adj):
        self.adj_mtx = new_adj
        
    def update_tgt_pos(self, new_pos):
        self.tgt_pos = new_pos if isinstance(new_pos, np.ndarray) else np.array(new_pos)
        self.tgt_pos_local = self.tgt_pos-self.tgt_pos[self.index_id]
        self.tgt_dist = np.linalg.norm(self.tgt_pos_local, axis=1)
        
    def get_ctrl(self, dist, dx=None):
        """_summary_
        ctrl = k * target_pos_local * adj_mtx * (dist - target_dist)
        Args:
            dist (list): distance between each drone
        """
        # print(self.target_pos_local)
        # print(self.adj_mtx[self.index_id, :])
        # print(dist)
        # print(self.target_dist)
        # TODO PID(PI)
        err = np.dot(np.transpose(self.tgt_pos_local),
                                    np.transpose(np.multiply( self.adj_mtx[self.index_id, :],
                                            (dist - self.tgt_dist))))
        if self.error_int is None:
            self.error_int = err
        else:
            self.error_int += err
        self.error_int[0] =  50 if self.error_int[0] >  50 else self.error_int[0]
        self.error_int[1] =  50 if self.error_int[1] >  50 else self.error_int[1]
        self.error_int[0] = -50 if self.error_int[0] < -50 else self.error_int[0]
        self.error_int[1] = -50 if self.error_int[1] < -50 else self.error_int[1]
        # vel_local = self.k_dist * np.dot(np.transpose(self.tgt_pos_local),
        #                             np.transpose(np.multiply( self.adj_mtx[self.index_id, :],
        #                                     (dist - self.tgt_dist))))
        vel_local = self.k_dist * err + self.ki_dist * self.error_int
        # print(dist - self.target_dist)
        vel_local_frd = [vel_local[1], vel_local[0]]
        return np.array(vel_local_frd)
        