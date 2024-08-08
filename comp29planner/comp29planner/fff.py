
from UavOnboard import UAVOnBoard
import os
import numpy as np
Uav = UAVOnBoard("config.json")
uav_id = os.environ.get('UAV_ID', '1')
uav_id = int(uav_id)
from NoopLoopUwb_python.handle_uwb import NoopLoopUWB
uwb = NoopLoopUWB(dev='/dev/ttyS4')
uav_id = 1
TARGET_POS_LOCAL = np.array([
    [0, 0],
    [1.8, 1.8],
    [3.6, 0],
    # [1.8, -1.8],
], dtype=np.float64)

"""
    2
  /   \
1------ 3
  \   /
    4

"""
ADJ_MTX = np.array([
#    1  2  3  4
    [0, 1, 1, 1],  # 1
    [1, 0, 1, 0],  # 2
    [1, 1, 0, 1],  # 3
    [1, 0, 1, 0],  # 4
], dtype=np.int64)

ADJ_MTX = np.array([
#    1  2  3
    [0, 1, 1],  # 1
    [1, 0, 1],  # 2
    [1, 1, 0],  # 3

], dtype=np.int64)


class FormationController:
    def __init__(self, target_pos, adj_mtx, uav_id_from_1, k=1) -> None:
        self.index_id = uav_id_from_1-1
        self.target_pos = target_pos
        self.adj_mtx = adj_mtx
        self.target_pos_local = self.target_pos-self.target_pos[self.index_id]
        self.target_dist = np.linalg.norm(self.target_pos_local, axis=1)
        print(self.target_pos_local)
        print(self.target_dist)
        self.k = k
        pass

    def update(self, dist):
        """_summary_
        ctrl = k * target_pos_local * adj_mtx * (dist - target_dist)
        Args:
            dist (list): distance between each drone
        """
        # print(self.target_pos_local)
        # print(self.adj_mtx[self.index_id, :])
        # print(dist)
        # print(self.target_dist)
        vel_local = self.k * np.dot(np.transpose(self.target_pos_local),
                                    np.transpose(np.multiply( self.adj_mtx[self.index_id, :],
                                            (dist - self.target_dist))))
        # print(dist - self.target_dist)
        vel_local_frd = [vel_local[1], vel_local[0]]
        return vel_local_frd
        print(vel_local)
        # print(np.multiply( self.adj_mtx[self.index_id, :],
        #                                     (dist - self.target_dist)))
        pass


ctrler = FormationController(TARGET_POS_LOCAL, ADJ_MTX, uav_id, k=0.1)
import time
Uav.start_send_vel_cmd_t()
while True:
    try:
        dist = uwb.get_filtered_dist_matrix()
        dist = dist[uav_id][1:4]
        print(f"dist: {dist}")
        vel_local_frd = ctrler.update(np.array(dist, dtype=np.float64))
        # Limit the values of vel_local_frd to -0.5 and 0.5
        vel_local_frd = np.clip(vel_local_frd, -0.5, 0.5)
        print(f"vel_local_frd: {vel_local_frd}")
        print()
        Uav.vel_set_frd.x = vel_local_frd[0]
        Uav.vel_set_frd.y = vel_local_frd[1]
        Uav.vel_set_frd.z = 0
        time.sleep(0.2)
    except KeyboardInterrupt:
        Uav.should_shutdown = True
        uwb.should_stop = True
        break