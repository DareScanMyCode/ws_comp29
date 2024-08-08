# coding: utf-8
import wheeltec_driver
from config import DrvCmd
import numpy as np
import math
def detect_t():
    ii = 0
    while True:
        car.detect_result.target_pos = [ii, ii+1, ii+2, ii+3]
        # print(f"car pose:{car.pose.x, car.pose.y}")
        ii+=1
        car.send_target_pose_img()
        # print("sent")
        time.sleep(0.2)
    pass

def car_goto_once(target_x, target_y, target_yaw):
    print(car.pos_source == car.POS_SRC_VICON)
    if car.pos_source == car.POS_SRC_VICON:
        car_pos = car.pose
        x,y = car_pos.x, car_pos.y
        yaw = car_pos.yaw
        p_star = np.array([target_x, target_y])
        p = np.array([x, y])
        dp = p_star - p
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        vx = np.linalg.norm(p_star - p) * np.matrix.dot(p_star - p, np.array([cy, sy]))
        vx = max(-0.3, min(vx, 0.4))
        if np.linalg.norm(dp) < 0.1:
            wz = target_yaw - yaw
        else:
            wz = (math.atan2(dp[1], dp[0]) - yaw)
        wz = max(-0.4, min(wz, 0.4))
        if math.fabs(vx) < 0.05:
            vx = 0
        if math.fabs(wz) < 0.05:
            wz = 0
        return vx, wz
    else:
        return 0, 0

TARGET_POSE_IMG_STAR = []
TARGET_SIZE_IMG_STAR = 300*400
def car_track_once(target_pose_img, target_pose_img_star, target_size_img_star):
    
    pass

def test_goto():
    global car
    car.pos_source = car.POS_SRC_VICON
    car_xyys = []
    car_xyy = np.array([car.pose.x, car.pose.y, car.pose.yaw], dtype=np.float32)
    for i in range(300):
        vx, wz = car_goto_once(10, 10, math.pi/2)
        print(f"在位置x={car.pose.x:5.2f}， {car.pose.y:5.2f}，{ car.pose.yaw:5.2f}，vx={vx}, wz={wz}")
        car_xyy += np.array([vx*math.cos(car_xyy[2]), vx*math.sin(car_xyy[2]), wz], dtype=np.float32) * 0.1
        car_xyy[2] = (car_xyy[2] +math.pi)% (2*math.pi)-math.pi
        car_xyys.append(car_xyy.copy())
        car.pose.x, car.pose.y, car.pose.yaw = car_xyy[0], car_xyy[1], car_xyy[2]
    # print(car_xyys)
    car.should_stop = True
    import matplotlib.pyplot as plt
    car_xyys = np.array(car_xyys)
    print(car_xyys.shape)
    # car_xyys.reshape(-1, 3)
    print(car_xyys[:, 0])
    plt.plot(car_xyys[:, 0], car_xyys[:, 1])
    plt.show()
import time
    
def car_control_t(car_: wheeltec_driver.WheeltecCar):
    while True:
        try:
            mis_state = car_.mission_state
            print(mis_state)
            # print( mis_state == DrvCmd.DRV_CMD_TASK_BEGIN)
            if mis_state == DrvCmd.DRV_CMD_TASK_WAIT:
                car_.send_vel(0, 0)
            if mis_state == DrvCmd.DRV_CMD_TASK_GOTO_WAIT:
                # car_control_once()
                vx, wz = car_goto_once(car.pose_goto[0], car.pose_goto[1], car.pose_goto[2])
                car.send_vel(vx, wz)
            elif mis_state == DrvCmd.DRV_CMD_TASK_BEGIN:
                # car_control_once()
                print("开始任务")
                vx, wz = car_goto_once(3,3, 0.2)
                car.send_vel(vx, wz)
                # car_.send_vel(0.2, 0.)
                # print("开始任务111")
            elif mis_state == DrvCmd.DRV_CMD_TASK_END:
                # print("结束任务")
                car_.send_vel(-0.0, 0)
            elif mis_state == DrvCmd.DRV_CMD_TASK_FOLLOW_PERSON:
                car_track_once(car.detect_result.target_pos, TARGET_POSE_IMG_STAR, TARGET_SIZE_IMG_STAR)
            time.sleep(0.1)
        except:
            time.sleep(0.1)
            pass

def car_control_once():
    mis_state = car.mission_state

car = wheeltec_driver.WheeltecCar()
if __name__ == "__main__":
    import threading
    car.send_vel(-0.3, 0.)
    time.sleep(5)
    car.send_vel(0,0)
    t = threading.Thread(target=car_control_t, args=(car,))
    t.start()
    
    t_detect = threading.Thread(target=detect_t )
    t_detect.start()
    # # test_goto()
    # car.send_vel(0, 0)
    while True:
        try:
            # car_control_once()
            time.sleep(1)
            
        except KeyboardInterrupt:
            car.should_stop = True
            break