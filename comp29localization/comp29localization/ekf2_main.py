# -*- coding: UTF-8 -*-
#!/usr/bin/python3
import os
import rclpy
import time
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from comp29msg.msg import CommunicationInfo, UAVInfo

import numpy as np
import scipy
from numpy import array, zeros
from colorama import Fore, Back, Style

from comp29localization.ekf_utils import dshot_2_T
from comp29localization.ekf_utils import quaternion2euler, euler2quaternion, constraint_yaw, constraint_float
from comp29localization.ekf_uwb_pose_alignment import optimize_pose_between_frames, optimize_pose_in_frames, local_pos_alginment

'''
    UAV Parameters
'''
uav_id  = 3
group_id = 4
acc_max = 5  # m / s^2
uav_m = 1.35
g = 9.81
dt = 0.001
#
swarm_num = 8
dist_array_num = 10
t_uwb_sampling = 1.5            # UWB以2秒为更新周期, 其中1.5秒等待, 0.5秒的区间用于接收数据                    # TODO 实际运算速率会低于这个值
t_uwb_recv_cutoff = 0.5         # 前者是为了保证有足够的运动距离, 不然测出来全是噪声
                                # 后者不能太宽也不能太窄, 在这个区间内收到的所有位置信息都会被记录且更新, 用于位置修正
#
x_initial = 0.
y_initial = 0.
'''
    Debugging options
'''
is_debugging_data_recv = False
is_debugging_ekf = False
is_debugging_uwb = False
is_display_data  = False


imu_data = Imu()
imu_data_last = Imu()
t_imu = float()
t_imu_last = float()
is_imu_data_ready = False
def imu_cb(msg:Imu):
    global imu_data, imu_data_last, t_imu, t_imu_last, is_imu_data_ready, is_debugging_data_recv
    #
    t_imu_last = float(t_imu)
    t_imu = time.time()
    #
    imu_data_last.angular_velocity.x = float(imu_data.angular_velocity.x)
    imu_data_last.angular_velocity.y = float(imu_data.angular_velocity.y)
    imu_data_last.angular_velocity.z = float(imu_data.angular_velocity.z)
    #
    imu_data_last.linear_acceleration.x = float(imu_data.linear_acceleration.x)
    imu_data_last.linear_acceleration.y = float(imu_data.linear_acceleration.y)
    imu_data_last.linear_acceleration.z = float(imu_data.linear_acceleration.z)
    #
    imu_data_last.orientation.w = float(imu_data.orientation.w)
    imu_data_last.orientation.x = float(imu_data.orientation.x)
    imu_data_last.orientation.y = float(imu_data.orientation.y)
    imu_data_last.orientation.z = float(imu_data.orientation.z)   
    #
    imu_data = msg
    #
    # 处理旋转方向
    rpy = quaternion2euler([imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z], is_degree=False)
    rpy[0] = rpy[0]                                                                     # TESTED
    rpy[1] = rpy[1]
    rpy[2] = rpy[2]
    quat_prime = euler2quaternion(rpy, is_degree=False)
    imu_data.orientation.w = quat_prime[0]                                              # 旋转到FRD坐标系下
    imu_data.orientation.x = quat_prime[1]                                              # TESTED, 换飞机的时候得检查
    imu_data.orientation.y = quat_prime[2]
    imu_data.orientation.z = quat_prime[3]
    #
    is_imu_data_ready = True

    if is_debugging_data_recv:
    #if True:
        print("[imu]")
        #print(imu_data)
        print("wx wy wz: %f %f %f"     % (imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z, ))
        print("ax ay az: %f %f %f"     % (imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, ))
        print("quat:     %f %f %f %f " % (imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z))
        #
        print("rpy:      %f %f %f" % (rpy[0] * 180. / np.pi, rpy[1] * 180. / np.pi, rpy[2] * 180. / np.pi, ))
        

servo_data = []
servo_data_last = []
t_servo = float()
t_servo_last = float()
is_servo_data_ready = False
def servo_cb(msg:Float32MultiArray):
    global servo_data, servo_data_last, t_servo, t_servo_last, is_servo_data_ready, is_debugging_data_recv
    #
    t_servo_last = float(t_servo)
    t_servo = time.time()    
    #
    servo_data_last.clear()
    for i in range(servo_data.__len__()):
        servo_data_last.append(servo_data[i])
    #
    servo_data = list(msg.data)
    is_servo_data_ready = True

    if is_debugging_data_recv:
        print("[servo]")
        print(servo_data)

vel_data = TwistStamped()
vel_data_last = TwistStamped()
t_vel = float()
t_vel_last = float()
is_vel_data_ready = False
def velocity_cb(msg:TwistStamped):
    global vel_data, vel_data_last, t_vel, t_vel_last, is_vel_data_ready, is_debugging_data_recv
    #
    t_vel_last = float(t_vel)
    t_vel = time.time()    
    #
    vel_data_last.twist.linear.x = float(vel_data.twist.linear.x)
    vel_data_last.twist.linear.y = float(vel_data.twist.linear.y)
    vel_data_last.twist.linear.z = float(vel_data.twist.linear.z)
    #
    vel_data_last.twist.angular.x = float(vel_data.twist.angular.x)
    vel_data_last.twist.angular.y = float(vel_data.twist.angular.y)
    vel_data_last.twist.angular.z = float(vel_data.twist.angular.z)
    #
    vel_data = msg
    is_vel_data_ready = True

    if is_debugging_data_recv:
        print("[vel]")
        print(vel_data)

local_pos_data = PoseStamped()
local_pos_data_last = PoseStamped()
t_local_pos = float()
t_local_pos_last = float()
is_local_pos_data_ready = False
def local_pos_cb(msg:PoseStamped):
    global local_pos_data, local_pos_data_last, t_local_pos, t_local_pos_last, is_local_pos_data_ready, is_debugging_data_recv
    #
    t_local_pos_last = float(t_local_pos)
    t_local_pos = time.time()   
    #
    local_pos_data_last.pose.position.x = float(local_pos_data.pose.position.x)
    local_pos_data_last.pose.position.y = float(local_pos_data.pose.position.y)
    local_pos_data_last.pose.position.z = float(local_pos_data.pose.position.z)
    #
    local_pos_data_last.pose.orientation.w = float(local_pos_data.pose.orientation.w)
    local_pos_data_last.pose.orientation.x = float(local_pos_data.pose.orientation.x)
    local_pos_data_last.pose.orientation.y = float(local_pos_data.pose.orientation.y)
    local_pos_data_last.pose.orientation.z = float(local_pos_data.pose.orientation.z)
    #
    local_pos_data = msg
    is_local_pos_data_ready = True

    if is_debugging_data_recv:
        print("[local position]")
        print(local_pos_data)

gps_data = PoseStamped()
gps_data_last = PoseStamped()
t_gps = float()
t_gps_last = float()
is_gps_data_ready = False
def gps_cb(msg:PoseStamped):
    global gps_data, gps_data_last, t_gps, t_gps_last, is_gps_data_ready, is_debugging_data_recv
    #
    t_gps_last = float(t_gps)
    t_gps = time.time()       
    #
    gps_data_last.pose.position.x = float(gps_data.pose.position.x)
    gps_data_last.pose.position.y = float(gps_data.pose.position.y)
    gps_data_last.pose.position.z = float(gps_data.pose.position.z)
    #    
    gps_data = msg
    is_gps_data_ready = True

    if is_debugging_data_recv:
        print("[gps]")
        print(gps_data)

vicon_data = PoseStamped()
vicon_data_last = PoseStamped()
t_vicon = float()
t_vicon_last = float()
is_vicon_data_ready = False
def vicon_cb(msg:PoseStamped):
    global vicon_data, vicon_data_last, t_vicon, t_vicon_last, is_vicon_data_ready, is_debugging_data_recv
    #
    t_vicon_last = float(t_vicon)
    t_vicon = time.time()     
    #
    vicon_data_last.pose.position.x = float(vicon_data.pose.position.x)
    vicon_data_last.pose.position.y = float(vicon_data.pose.position.y)
    vicon_data_last.pose.position.z = float(vicon_data.pose.position.z)
    #
    vicon_data_last.pose.orientation.w = float(vicon_data.pose.orientation.w)
    vicon_data_last.pose.orientation.x = float(vicon_data.pose.orientation.x)
    vicon_data_last.pose.orientation.y = float(vicon_data.pose.orientation.y)
    vicon_data_last.pose.orientation.z = float(vicon_data.pose.orientation.z)
    #
    vicon_data = msg
    is_vicon_data_ready = True

    if is_debugging_data_recv:
        print("[vicon]")
        print(vicon_data)

comm_info = CommunicationInfo()
is_comm_info_updated = False
def comm_info_cb(msg:CommunicationInfo):
    #
    global comm_info, is_comm_info_updated
    #
    comm_info = msg
    is_comm_info_updated = True

this_uav_uwb_data = Float32MultiArray()
is_this_uav_uwb_updated = False
def this_uav_uwb_cb(msg:Float32MultiArray):
    #
    global this_uav_uwb_data, is_this_uav_uwb_updated
    #
    this_uav_uwb_data = msg
    is_this_uav_uwb_updated = True

def EKF_prediction(Xk, Pk, acc, acc_last, gyro, gyro_last, dt, is_debugging=False):
    # \dot X    = V
    # \dot V    = DCM @ acc_imu
    # \dot quat = DCM @ gyro_imu
    #
    # i.e.,
    # X    = X    + V * dt
    # V    = V    + DCM @ acc_imu
    # quat = quat + DCM @ gyro_imu      其中acc_imu和gyro_imu是输入, 根据这个迭代公式取计算

    #
    # parameters
    global acc_max
    k_acc_x = 1e-2
    k_acc_y = 1e-2
    k_acc_z = 1e-2
    k_gyro_x = 1e-5
    k_gyro_y = 1e-5
    k_gyro_z = 1e-5

    #
    # Step 1
    # update dynamics
    increment = np.zeros(10)
    # x, y, z
    increment[0] = Xk[3] * dt
    increment[1] = Xk[4] * dt
    increment[2] = Xk[5] * dt
    # vx, vy, vz
    increment[3] = acc[0] * dt
    increment[4] = acc[1] * dt
    increment[5] = acc[2] * dt
    #
    q0k = Xk[6];     q1k = Xk[7];     q2k = Xk[8];    q3k = Xk[9];
    wx  = gyro[0];   wy  = gyro[1];   wz  = gyro[2];
    increment[6] = (q0k * 0 - q1k * wx - q2k * wy - q3k * wz) * dt
    increment[7] = (q1k * 0 + q0k * wx - q3k * wy + q2k * wz) * dt
    increment[8] = (q2k * 0 + q3k * wx + q0k * wy - q1k * wz) * dt
    increment[9] = (q3k * 0 - q2k * wx + q1k * wy + q0k * wz) * dt             # 捷联的模型

    #
    for xyz_t in range(3, 6):
        increment[xyz_t] = constraint_float(increment[xyz_t], -acc_max, acc_max)                # 滤波器不一定100%靠谱, 所以只能加个这个限制

    #
    Xk_prime = Xk + increment

    #
    # Step 2
    # update covarance
    I33 = np.identity(3)
    I44 = np.identity(4)
    zero33 = np.zeros([3, 3])
    zero34 = np.zeros([3, 4])
    zero43 = np.zeros([4, 3])
    DCM_B_N = np.matrix([[q0k ** 2 + q1k ** 2 - q2k ** 2 - q3k ** 2, 2 * (q1k * q2k - q0k *q3k),                2 * (q1k * q3k + q0k *q2k)],
                         [2 * (q1k * q2k + q0k *q3k),                q0k ** 2 - q1k ** 2 + q2k ** 2 - q3k ** 2, 2 * (q2k * q3k - q0k *q1k)],
                         [2 * (q1k * q3k - q0k *q2k),                2 * (q2k * q3k + q0k *q1k),                q0k ** 2 - q1k ** 2 - q2k ** 2 + q3k ** 2]])

    #
    #
    F_V_P = np.identity(3) * dt
    #
    F_V_V = np.identity(3)                          # 这个我不是太确定, 但是确实是和四元数以及加速度无关
    #
    PHI1 = np.matrix([[ q0k, -q3k,  q2k],
                      [ q1k,  q2k,  q3k],
                      [-q2k,  q1k,  q0k],
                      [-q3k, -q0k,  q1k]])
    PHI2 = np.matrix([[ q3k,  q0k, -q1k],
                      [ q2k, -q1k, -q0k],
                      [ q1k,  q2k,  q3k],
                      [ q0k, -q3k,  q2k]])
    PHI3 = np.matrix([[-q2k,  q1k,  q0k],
                      [ q3k,  q0k, -q1k],
                      [-q0k,  q3k, -q2k],
                      [ q1k,  q2k,  q3k]])
    delta_v = np.matrix([acc[0] * dt, acc[1] * dt, acc[2] * dt]).reshape([3, 1])
    F_Q_V = np.block([[np.transpose(PHI1 @ delta_v)],
                      [np.transpose(PHI2 @ delta_v)],
                      [np.transpose(PHI3 @ delta_v)]])
    #
    M_w_2 = np.matrix([[0,           -wx / 2 * dt, -wy / 2 * dt, -wz / 2 * dt],
                       [wx / 2 * dt, 0,             wz / 2 * dt, -wy / 2 * dt],
                       [wy / 2 * dt, -wz / 2 * dt, 0,             wx / 2 * dt],         # TODO check it out
                       [wz / 2 * dt,  wy / 2 * dt, -wx / 2 * dt, 0]])
    F_Q_Q = np.identity(4) + M_w_2
    #
    Fx = np.block([[I33,    F_V_P,  zero34],
                   [zero33, F_V_V,  F_Q_V],
                   [zero43, zero43, F_Q_Q]])

    #
    # step 3
    # calculate noise
    #
    F_G_Q = np.matrix([[-q1k / 2, -q2k / 2, -q3k / 2],
                       [ q0k / 2, -q3k / 2,  q2k / 2],
                       [ q3k / 2,  q0k / 2, -q1k / 2],
                       [-q2k / 2,  q1k / 2,  q0k / 2]])
    #
    F_A_V = DCM_B_N
    Fu = np.block([[zero33, zero33],
                   [F_A_V,  zero33],
                   [zero43, F_G_Q]])
    #
    Q = np.zeros([6, 6])
    Q[0, 0] = k_acc_x * (acc[0] - acc_last[0])
    Q[1, 1] = k_acc_y * (acc[1] - acc_last[1])
    Q[2, 2] = k_acc_z * (acc[2] - acc_last[2])
    Q[3, 3] = k_gyro_x * (gyro[0] - gyro_last[0])
    Q[4, 4] = k_gyro_y * (gyro[1] - gyro_last[1])
    Q[5, 5] = k_gyro_z * (gyro[2] - gyro_last[2])

    Phat_1 = Fx @ Pk @ np.transpose(Fx)
    Phat_2 = Fu @ Q  @ np.transpose(Fu)
    Phat = Phat_1 + Phat_2

    if is_debugging:
        Fx_debug     = np.asarray(Fx)
        Fu_debug     = np.asarray(Fu)
        Phat_1_debug = np.asarray(Phat_1)
        Phat_2_debug = np.asarray(Phat_2)
        Phat_debug   = np.asarray(Phat)
        #
        # print("[EKF prediction]")
        # print("vel: %f %f %f x dt: %f" % (Xk[3], Xk[4], Xk[5], dt))
        #print("increment")
        #print(increment)

    return Xk_prime, Phat

def EKF_update_fcu_velocity(Xk, Pk, vel, vel_last, dt, is_debugging=False):
    #
    # parameters
    k_vx = 5e-3
    k_vy = 5e-3
    k_vz = 5e-3

    # 光流速度
    Hx = np.zeros([3, 10])
    Hx[0, 3] = 1
    Hx[1, 4] = 1
    Hx[2, 5] = 1

    #
    R = np.identity(3) * np.array([k_vx * (vel[0] - vel_last[0]) ** 2,
                                   k_vy * (vel[1] - vel_last[1]) ** 2,
                                   k_vz * (vel[2] - vel_last[2]) ** 2])

    #
    Hv = np.identity(3)

    #
    innov_1 = Hx @ Pk @ np.transpose(Hx)
    innov_2 = Hv @ R @ np.transpose(Hv)
    innov = innov_1 + innov_2
    try:
        inv_innov = np.linalg.inv(innov)
    except:
        print("[FCU Velocity] no proper K")
        K = np.zeros([10, 3])
        return Xk, K, Pk
    #
    K = Pk @ np.transpose(Hx) @ inv_innov

    #
    delta_z = np.array([vel[0] - Xk[3],
                        vel[1] - Xk[4],
                        vel[2] - Xk[5]])
    K_delta_z = K @ delta_z
    K_delta_z = np.asarray(K_delta_z).reshape(1, 10)[0]        # 这里故意用行向量, 这里矩阵维数有问题处理一下
    X = Xk + K_delta_z

    #
    # Pt_2 = K @ innov @ np.transpose(K)
    # Pt = P_hat - Pt_2
    Pt = Pk - K @ Hx @ Pk

    if is_debugging:
        if abs(X[3]) > 5. or abs(X[4]) > 5.:
            print("[FCU Velocity] K = ")
            print(K)
            print("[FCU Velocity] increment")
            print("vel: %f %f %f x dt: %f" % (vel[0], vel[1], vel[2], dt))
            print("[FCU Velocity] delta_z")
            print(delta_z)
            print("[FCU Velocity] K x delta_z")
            print(K_delta_z)        
            print("innov 1")
            print(innov_1)
            print("innov 2")
            print(innov_2)

        vel_debug      = np.array(vel)
        vel_last_debug = np.array(vel_last)
        delta_z_debug = np.asarray(delta_z)
        Xk_debug = np.asarray(Xk)
        X_debug  = np.asarray(X)
        #
        K_debug = np.asarray(K)


    return X, K, Pt

def EKF_update_attitude(Xk, Pk, rpy, rpy_last, quat, quat_last, dt, is_debugging=False):
    #
    # parameters
    k_q0 = 1e-15         # 这个其实可以调小一点, 10^-10到10^-20都可以反而准确, 这里调大是为了验证理论正确性
    k_q1 = 1e-15
    k_q2 = 1e-15
    k_q3 = 1e-15

    if rpy != None and rpy_last != None:
        #
        quat      = euler2quaternion(rpy, is_degree=False)
        quat_last = euler2quaternion(rpy_last, is_degree=False)
        if is_debugging:
            rpy_input_4_checking      = quaternion2euler(quat,      is_degree=True)
            rpy_input_last_4_checking = quaternion2euler(quat_last, is_degree=True)
            rpy_4_checking            = quaternion2euler([Xk[6], Xk[7], Xk[8], Xk[9]], is_degree=True)
    else:
        if is_debugging:
            # print("[FCU attitude] quat")
            # print(quat)
            # print("[FCU attitude] quat converted rpy")
            # print(quaternion2euler(quat))
            pass

    # 光流速度
    Hx = np.zeros([4, 10])
    Hx[0, 6] = 1
    Hx[1, 7] = 1
    Hx[2, 8] = 1
    Hx[3, 9] = 1

    #
    R = np.identity(4) * np.array([k_q0 * (quat[0] - quat_last[0]) ** 2,
                                   k_q1 * (quat[1] - quat_last[1]) ** 2,
                                   k_q2 * (quat[2] - quat_last[2]) ** 2,
                                   k_q3 * (quat[2] - quat_last[2]) ** 2,])

    #
    Hv = np.identity(4)

    #
    innov_1 = Hx @ Pk @ np.transpose(Hx)
    innov_2 = Hv @ R @ np.transpose(Hv)
    innov = innov_1 + innov_2
    try:
        inv_innov = np.linalg.inv(innov)
    except:
        print("[FCU Attitude] no proper K")
        K = np.zeros([10, 4])
        return Xk, K, Pk
    #
    K = Pk @ np.transpose(Hx) @ inv_innov

    #
    delta_z = np.array([quat[0] - Xk[6],
                        quat[1] - Xk[7],
                        quat[2] - Xk[8],
                        quat[3] - Xk[9]])
    K_delta_z = K @ delta_z
    K_delta_z = np.asarray(K_delta_z).reshape(1, 10)[0]        # 这里故意用行向量, 这里矩阵维数有问题处理一下
    X = Xk + K_delta_z

    #
    # Pt_2 = K @ innov @ np.transpose(K)
    # Pt = P_hat - Pt_2
    Pt = Pk - K @ Hx @ Pk

    if is_debugging:
        # print("[FCU Attitude] K = ")
        # print(K)
        # print("[FCU Attitude] delta Z")
        # print(delta_z)
        # print("[FCU Attitude] K x delta Z")
        # print(K_delta_z)

        quat_debug    = np.array(quat)
        delta_z_debug = np.asarray(delta_z)
        Xk_debug = np.asarray(Xk)
        X_debug  = np.asarray(X)
        #
        K_debug = np.asarray(K)


    return X, K, Pt

vx_slam_history = []
vy_slam_history = []
vz_slam_history = []
def EKF_update_SLAM(Xk, Pk, xyz_ned, xyz_ned_last, dt, is_debugging=False):
    #
    # parameters
    k_vx = 1e-5
    k_vy = 1e-5
    k_vz = 1e-5
    #
    N = 16
    fc = 50.  # cutoff freq
    fs = 400.  # samping freq
    omega_c = 0.025  # 2 * fc / fs
    max_len = 12 * N

    bf1, af1 = scipy.signal.butter(N, omega_c, btype='low')
    bf2, af2 = scipy.signal.butter(N, omega_c, btype='low')
    bf3, af3 = scipy.signal.butter(N, omega_c, btype='low')

    vel_x_t = (xyz_ned[0] - xyz_ned_last[0]) / dt
    vel_y_t = (xyz_ned[1] - xyz_ned_last[1]) / dt
    vel_z_t = (xyz_ned[2] - xyz_ned_last[2]) / dt

    if vx_slam_history.__len__() > max_len:
        vx_slam_history.pop(-1)
        vy_slam_history.pop(-1)
        vz_slam_history.pop(-1)
    vx_slam_history.append(vel_x_t)
    vy_slam_history.append(vel_y_t)
    vz_slam_history.append(vel_z_t)

    if len(vx_slam_history) < 4 * N:
        vel_x_t = float(scipy.signal.lfilter(bf1, af1, array(vx_slam_history))[-1])
        vel_y_t = float(scipy.signal.lfilter(bf2, af2, array(vy_slam_history))[-1])
        vel_z_t = float(scipy.signal.lfilter(bf3, af3, array(vz_slam_history))[-1])
    else:
        vel_x_t = float(scipy.signal.filtfilt(bf1, af1, array(vx_slam_history))[-1])
        vel_y_t = float(scipy.signal.filtfilt(bf2, af2, array(vy_slam_history))[-1])
        vel_z_t = float(scipy.signal.filtfilt(bf3, af3, array(vz_slam_history))[-1])

    vel_x_last = vx_slam_history[vx_slam_history.__len__() - 2]
    vel_y_last = vy_slam_history[vy_slam_history.__len__() - 2]
    vel_z_last = vz_slam_history[vz_slam_history.__len__() - 2]

    #
    Hx = np.zeros([3, 10])
    Hx[0, 3] = 1
    Hx[1, 4] = 1
    Hx[2, 5] = 1

    #
    R = np.identity(3) * np.array([k_vx * (vel_x_t - vel_x_last) ** 2,
                                   k_vy * (vel_y_t - vel_y_last) ** 2,
                                   k_vz * (vel_z_t - vel_z_last) ** 2])

    #
    Hv = np.identity(3)

    #
    innov_1 = Hx @ Pk @ np.transpose(Hx)
    innov_2 = Hv @ R @ np.transpose(Hv)
    innov = innov_1 + innov_2
    try:
        inv_innov = np.linalg.inv(innov)
    except:
        print("[SLAM] no proper K")
        K = np.zeros([10, 3])
        return Xk, K, Pk
    #
    K = Pk @ np.transpose(Hx) @ inv_innov

    #
    delta_z = np.array([vel_x_t - Xk[3],
                        vel_y_t - Xk[4],
                        vel_z_t - Xk[5]])
    K_delta_z = K @ delta_z
    K_delta_z = np.asarray(K_delta_z).reshape(1, 10)[0]        # 这里故意用行向量, 这里矩阵维数有问题处理一下
    X = Xk + K_delta_z

    #
    # Pt_2 = K @ innov @ np.transpose(K)
    # Pt = P_hat - Pt_2
    Pt = Pk - K @ Hx @ Pk

    if is_debugging:
        print("[SLAM] K = ")
        print(K)

        vel_debug      = np.array([vel_x_t, vel_y_t, vel_z_t])
        vel_last_debug = np.array([vel_x_last, vel_y_last, vel_z_last])
        delta_z_debug = np.asarray(delta_z)
        Xk_debug = np.asarray(Xk)
        X_debug  = np.asarray(X)
        #
        K_debug = np.asarray(K)

    return X, K, Pt



acc_Tx_history = []
acc_Ty_history = []
acc_Tz_history = []
def EKF_update_servo(Xk, Pk, servo, servo_last, dt, rpy=None, is_debugging=False):
    #
    # 根据牛顿第二定律,
    # ax                            vx
    # ay = (R * \Sum T) / m - k_f * vy -
    # az                            vz   g
    # 其实这个模型不太准确, 因为没有考虑风速, 但是凑合吧, 这里直接
    # 所以H_x =
    # \partial P / \partial V_measurement       \partial V / \partial V_measurement     \partial Q / \partial V_measurement
    # 这里可以暴力一点
    # \partial P / \partial V_measurement = 0
    # \partial V / \partial V_measurement = DCM_B_N
    # \partial Q / \partial V_measurement = 0

    #
    # parameters
    global uav_m, acc_max
    k_vx = 1e-1
    k_vy = 1e-1
    k_vz = 1e-1
    #
    N = 4
    fc = 50.  # cutoff freq
    fs = 400.  # samping freq
    omega_c = 0.35  # 2 * fc / fs
    max_len = 12 * N

    bf1, af1 = scipy.signal.butter(N, omega_c, btype='low')
    bf2, af2 = scipy.signal.butter(N, omega_c, btype='low')
    bf3, af3 = scipy.signal.butter(N, omega_c, btype='low')

    if rpy == None:
        # 使用内部旋转矩阵
        q0k = Xk[6]
        q1k = Xk[7]
        q2k = Xk[8]
        q3k = Xk[9]
    else:
        # 使用外部旋转矩阵
        quat = euler2quaternion(rpy)
        q0k = quat[0]
        q1k = quat[1]
        q2k = quat[2]
        q3k = quat[3]

    #
    DCM_B_N = np.matrix([[q0k ** 2 + q1k ** 2 - q2k ** 2 - q3k ** 2, 2 * (q1k * q2k - q0k * q3k), 2 * (q1k * q3k + q0k * q2k)],
                         [2 * (q1k * q2k + q0k * q3k), q0k ** 2 - q1k ** 2 + q2k ** 2 - q3k ** 2, 2 * (q2k * q3k - q0k * q1k)],
                         [2 * (q1k * q3k - q0k * q2k), 2 * (q2k * q3k + q0k * q1k), q0k ** 2 - q1k ** 2 - q2k ** 2 + q3k ** 2]])

    # servo -> acc
    T = np.zeros(4)
    T[0] = dshot_2_T(servo[0])
    T[1] = dshot_2_T(servo[1])
    T[2] = dshot_2_T(servo[2])
    T[3] = dshot_2_T(servo[3])
    #
    acc_ned = np.zeros(3)
    acc_ned[0] = (T[0] + T[1] + T[2] + T[3]) * (2 * (q1k * q3k - q0k * q2k)) / uav_m
    acc_ned[1] = (T[0] + T[1] + T[2] + T[3]) * (2 * (q2k * q3k + q0k * q1k)) / uav_m
    acc_ned[2] = (T[0] + T[1] + T[2] + T[3]) * (q0k ** 2 - q1k ** 2 - q2k ** 2 + q3k ** 2)  / uav_m

    T_last = np.zeros(4)
    T_last[0] = dshot_2_T(servo_last[0])
    T_last[1] = dshot_2_T(servo_last[1])
    T_last[2] = dshot_2_T(servo_last[2])
    T_last[3] = dshot_2_T(servo_last[3])
    acc_ned_last = np.zeros(3)
    acc_ned_last[0] = (T_last[0] + T_last[1] + T_last[2] + T_last[3]) * (2 * (q1k * q3k - q0k * q2k)) / uav_m
    acc_ned_last[1] = (T_last[0] + T_last[1] + T_last[2] + T_last[3]) * (2 * (q2k * q3k + q0k * q1k)) / uav_m
    acc_ned_last[2] = (T_last[0] + T_last[1] + T_last[2] + T_last[3]) * (q0k ** 2 - q1k ** 2 - q2k ** 2 + q3k ** 2)  / uav_m

    if acc_Tx_history.__len__() > max_len:
        acc_Tx_history.pop(-1)
        acc_Ty_history.pop(-1)
        acc_Tz_history.pop(-1)
    acc_Tx_history.append(acc_ned[0])
    acc_Ty_history.append(acc_ned[1])
    acc_Tz_history.append(acc_ned[2])

    # LPF & constraints
    if len(acc_Tx_history) < 4 * N:
        acc_ned[0] = float(scipy.signal.lfilter(bf1, af1, array(acc_Tx_history))[-1])
        acc_ned[1] = float(scipy.signal.lfilter(bf2, af2, array(acc_Ty_history))[-1])
        acc_ned[2] = float(scipy.signal.lfilter(bf3, af3, array(acc_Tz_history))[-1])
    else:
        acc_ned[0] = float(scipy.signal.filtfilt(bf1, af1, array(acc_Tx_history))[-1])
        acc_ned[1] = float(scipy.signal.filtfilt(bf2, af2, array(acc_Ty_history))[-1])
        acc_ned[2] = float(scipy.signal.filtfilt(bf3, af3, array(acc_Tz_history))[-1])
    #
    for xyz_t in range(0, 3):
        acc_ned[xyz_t] = constraint_float(acc_ned[xyz_t], -acc_max, acc_max)                # 滤波器不一定100%靠谱, 所以只能加个这个限制

    #
    #
    zero33 = np.zeros([3, 3])
    zero34 = np.zeros([3, 4])
    Hx = np.block([zero33, DCM_B_N, zero34])

    #
    R = np.identity(3) * np.array([k_vx * (acc_ned[0] - acc_ned_last[0]) ** 2,
                                   k_vy * (acc_ned[1] - acc_ned_last[1]) ** 2,
                                   k_vz * (acc_ned[2] - acc_ned_last[2]) ** 2])

    #
    Hv = np.identity(3)

    #
    innov_1 = Hx @ Pk @ np.transpose(Hx)
    innov_2 = Hv @ R @ np.transpose(Hv)
    innov = innov_1 + innov_2
    try:
        inv_innov = np.linalg.inv(innov)
    except:
        print("[Servo] no proper K")
        K = np.zeros([10, 3])
        return Xk, K, Pk
    #
    K = Pk @ np.transpose(Hx) @ inv_innov

    #
    if T[0] + T[1] + T[2] + T[3] >= 0.:
        delta_z = np.array([acc_ned[0] * dt - Xk[3],
                            acc_ned[1] * dt - Xk[4],
                            acc_ned[2] * dt - g - Xk[5]])
    else:
        delta_z = np.array([acc_ned[0] * dt - Xk[3],
                            acc_ned[1] * dt - Xk[4],
                            acc_ned[2] * dt - Xk[5]])               # TODO 要不要减去重力加速度g?
                                                                    # 其实应该是要的, 这里通过电机转动判断飞机是否起飞, 起飞以后应该就是减去g反而是准的

    K_delta_z = K @ delta_z
    K_delta_z = np.asarray(K_delta_z).reshape(1, 10)[0]  # 这里故意用行向量, 这里矩阵维数有问题处理一下
    X = Xk + K_delta_z

    #
    # Pt_2 = K @ innov @ np.transpose(K)
    # Pt = P_hat - Pt_2
    Pt = Pk - K @ Hx @ Pk

    if is_debugging:
        print("[Servo] K = ")
        print(K)
        print("[servo] delta_z = ")
        print(delta_z)
        print("[servo] K x delta_z = ")
        print(K_delta_z)        

        vel_debug = np.array(acc_ned)
        vel_last_debug = np.array(acc_ned_last)
        delta_z_debug = np.asarray(delta_z)
        Xk_debug = np.asarray(Xk)
        X_debug = np.asarray(X)
        #
        K_debug = np.asarray(K)

    return X, K, Pt

pos_x_history = []
pos_y_history = []
pos_z_history = []  # 如果是真机这个变量要定长, 超过长度则压一个弹一个
def pos_filter(x, y, z, dt=0.1):
    global pos_x_history, pos_y_history, pos_z_history

    #
    # parameters
    N = 4
    fc = 50.  # cutoff freq
    fs = 400.  # samping freq
    omega_c = 0.35  # 2 * fc / fs
    max_len = 12 * N

    bf1, af1 = scipy.signal.butter(N, omega_c, btype='low')
    bf2, af2 = scipy.signal.butter(N, omega_c, btype='low')
    bf3, af3 = scipy.signal.butter(N, omega_c, btype='low')

    if pos_x_history.__len__() > max_len:
        pos_x_history.pop(-1)
        pos_y_history.pop(-1)
        pos_z_history.pop(-1)
    pos_x_history.append(x)
    pos_y_history.append(y)
    pos_z_history.append(z)

    if len(pos_x_history) < 4 * N:
        x = float(scipy.signal.lfilter(bf1, af1, array(pos_x_history))[-1])
        y = float(scipy.signal.lfilter(bf2, af2, array(pos_y_history))[-1])
        z = float(scipy.signal.lfilter(bf3, af3, array(pos_z_history))[-1])
    else:
        x = float(scipy.signal.filtfilt(bf1, af1, array(pos_x_history))[-1])
        y = float(scipy.signal.filtfilt(bf2, af2, array(pos_y_history))[-1])
        z = float(scipy.signal.filtfilt(bf3, af3, array(pos_z_history))[-1])

    return x, y, z

vel_x_history = []
vel_y_history = []
vel_z_history = []  # 如果是真机这个变量要定长, 超过长度则压一个弹一个
def vel_filter(vx, vy, vz, dt=0.1):
    global vel_x_history, vel_y_history, vel_z_history

    #
    # parameters
    N = 4
    fc = 50.  # cutoff freq
    fs = 400.  # samping freq
    omega_c = 0.35  # 2 * fc / fs
    max_len = 12 * N

    bf1, af1 = scipy.signal.butter(N, omega_c, btype='low')
    bf2, af2 = scipy.signal.butter(N, omega_c, btype='low')
    bf3, af3 = scipy.signal.butter(N, omega_c, btype='low')

    if vel_x_history.__len__() > max_len:
        vel_x_history.pop(-1)
        vel_y_history.pop(-1)
        vel_z_history.pop(-1)
    vel_x_history.append(vx)
    vel_y_history.append(vy)
    vel_z_history.append(vz)

    if len(vel_x_history) < 4 * N:
        vx = float(scipy.signal.lfilter(bf1, af1, array(vel_x_history))[-1])
        vy = float(scipy.signal.lfilter(bf2, af2, array(vel_y_history))[-1])
        vz = float(scipy.signal.lfilter(bf3, af3, array(vel_z_history))[-1])
    else:
        vx = float(scipy.signal.filtfilt(bf1, af1, array(vel_x_history))[-1])
        vy = float(scipy.signal.filtfilt(bf2, af2, array(vel_y_history))[-1])
        vz = float(scipy.signal.filtfilt(bf3, af3, array(vel_z_history))[-1])

    return vx, vy, vz

acc_x_history = []
acc_y_history = []
acc_z_history = []                      # 如果是真机这个变量要定长, 超过长度则压一个弹一个
def acc_filter(ax, ay, az, dt=0.1):
    global acc_x_history, acc_y_history, acc_z_history

    #
    # parameters
    N = 8
    fc = 50.  # cutoff freq
    fs = 400.  # samping freq
    omega_c = 0.05  # 2 * fc / fs
    max_len = 24 * N                    # TESTED 滤波器的阶段频率和阶数尽可能小, 但是如果频繁出现极大值, 则反而需要提高阈值

    bf1, af1 = scipy.signal.butter(N, omega_c, btype='low')
    bf2, af2 = scipy.signal.butter(N, omega_c, btype='low')
    bf3, af3 = scipy.signal.butter(N, omega_c, btype='low')

    if acc_x_history.__len__() > max_len:
        acc_x_history.pop(-1)
        acc_y_history.pop(-1)
        acc_z_history.pop(-1)
    acc_x_history.append(ax)
    acc_y_history.append(ay)
    acc_z_history.append(az)

    if len(acc_x_history) < 4 * N:
        ax = float(scipy.signal.lfilter(bf1, af1, array(acc_x_history))[-1])
        ay = float(scipy.signal.lfilter(bf2, af2, array(acc_y_history))[-1])
        az = float(scipy.signal.lfilter(bf3, af3, array(acc_z_history))[-1])
    else:
        ax = float(scipy.signal.filtfilt(bf1, af1, array(acc_x_history))[-1])
        ay = float(scipy.signal.filtfilt(bf2, af2, array(acc_y_history))[-1])
        az = float(scipy.signal.filtfilt(bf3, af3, array(acc_z_history))[-1])

    return ax, ay, az

def load_system_params(node):
    #
    global uav_id, group_id, swarm_num, dist_array_num
    global x_initial, y_initial
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

        x_initial          = mis_cfg["INITIAL_POS"]['UAV' + str(uav_id)][0]
        y_initial          = mis_cfg["INITIAL_POS"]['UAV' + str(uav_id)][1]        

        group_id = 1 if uav_id in group_1 else 2 if uav_id in group_2 else 3 if uav_id in group_3 else -1
        
        if group_id == -1:
            node.get_logger().warn("[fsm] UAV_ID未被分组！！！！")
        
        #dist_array_num = swarm_num + 1
            
    else:
        node.get_logger().warn("[fsm] 未能读取到任务配置文件！！！！")
        dist_array_num = 10

def main(args=None):
    #
    global imu_data,       imu_data_last,       t_imu,       t_imu_last,       is_imu_data_ready
    global servo_data,     servo_data_last,     t_servo,     t_servo_last,     is_servo_data_ready
    global vel_data,       vel_data_last,       t_vel,       t_vel_last,       is_vel_data_ready
    global local_pos_data, local_pos_data_last, t_local_pos, t_local_pos_last, is_local_pos_data_ready
    global gps_data,       gps_data_last,       t_gps,       t_gps_last,       is_gps_data_ready
    global vicon_data,     vicon_data_last,     t_vicon,     t_vicon_last,     is_vicon_data_ready
    global comm_info,         is_comm_info_updated
    global this_uav_uwb_data, is_this_uav_uwb_updated
    #
    global uav_id, group_id, swarm_num, dist_array_num, t_uwb_sampling, t_uwb_recv_cutoff
    global x_initial, y_initial
    global dt

    rclpy.init(args=args)
    node = Node('ekf2_main_w_uwb')

    load_system_params(node)

    #
    imu_sub             = node.create_subscription(Imu, 'imu', imu_cb, 10)
    servo_sub           = node.create_subscription(Float32MultiArray, 'servo_data', servo_cb, 1)
    velocity_sub        = node.create_subscription(TwistStamped, 'velocity_and_angular', velocity_cb, 1)
    #
    local_pos_sub       = node.create_subscription(PoseStamped, 'local_position_ned', local_pos_cb, 1)
    gps_pos_sub         = node.create_subscription(PoseStamped, 'gps_position', gps_cb, 1)
    vicon_pos_sub       = node.create_subscription(PoseStamped, '/vicon_pose', vicon_cb, 1)
    #
    this_uav_uwb_topic_name       = '/uwb_filtered'
    this_uav_uwb_sub     = node.create_subscription(Float32MultiArray, this_uav_uwb_topic_name, this_uav_uwb_cb, 1)
    comm_info_topic_name = "/uav" + str(uav_id) + "/comm_info"
    uav_swarm_uwb_sub    = node.create_subscription(CommunicationInfo, comm_info_topic_name, comm_info_cb, 1)
    #
    ekf2_pos_topic_name = '/uav' + str(uav_id) + "/ekf2/pose"
    ekf2_pos_pub        = node.create_publisher(PoseStamped,  ekf2_pos_topic_name, 1)
    ekf2_vel_topic_name = '/uav' + str(uav_id) + "/ekf2/vel"    
    ekf2_vel_pub        = node.create_publisher(TwistStamped, ekf2_vel_topic_name, 1)
    ekf2_pose = PoseStamped()
    ekf2_vel  = TwistStamped()
    #
    ekf2_uwb_local_pos_topic_name  = '/uav' + str(uav_id) + "/ekf2/local_pose"
    ekf2_uwb_local_pos_pub         = node.create_publisher(PoseStamped,  ekf2_uwb_local_pos_topic_name, 1)
    ekf2_uwb_center_pos_topic_name = '/uav' + str(uav_id) + "/ekf2/center_pos"
    ekf2_uwb_center_pos_pub        = node.create_publisher(PoseStamped,  ekf2_uwb_center_pos_topic_name, 1)
    ekf2_uwb_local_pos  = PoseStamped()
    ekf2_uwb_center_pos = PoseStamped()

    print((Fore.LIGHTBLUE_EX + "[ekf2 uwb] Initialized success, UAV id: %d, swarm num: %d ..." % (uav_id, swarm_num) + Style.RESET_ALL))

    #
    t0 = time.time()
    index = 0

    ''' 
        initialization
    '''
    # X = [ x, y, z, vx, vy, vz, q0, q1, q2, q3 ]
    time.sleep(1.5)                     # wait for imu ready
    X = np.zeros(10)                    # 这里故意用行向量
    X[6] = 1                            # 四元数标准化
    P = np.identity(10) * 1.

    while not is_imu_data_ready:
        node.get_logger().info("[ekf2] waiting for imu data ...")
        rclpy.spin_once(node)
        time.sleep(1.0)
    print(Style.BRIGHT + Fore.GREEN + "[ekf2] Data Received, Starting EKF ..." + Style.RESET_ALL)
    #
    # inject initial value if possible
    if is_imu_data_ready:
        if not (imu_data.orientation.w == 0 and imu_data.orientation.x == 0 and imu_data.orientation.y == 0 and imu_data.orientation.z == 0):
        #if False:                                                                           # TODO, 检查时用
            X[6] = imu_data.orientation.w                                              # CRITICAL, 初始点在平衡点上会有很大帮助
            X[7] = imu_data.orientation.x
            X[8] = imu_data.orientation.y
            X[9] = imu_data.orientation.z
            #
            is_imu_data_ready = False
        else:
            X[6] = 1.           # q0 or w
            X[7] = 0.
            X[8] = 0.
            X[9] = 0.
            print("[ekf2] using default quaternion ...")
    if is_vel_data_ready:
        X[3] = vel_data.twist.linear.x                                                 # CTITICAL
        X[4] = vel_data.twist.linear.y
        X[5] = vel_data.twist.linear.z
        #
        is_vel_data_ready = False
    #if is_local_pos_data_ready:
    if False:
        if local_pos_data.pose.position.x != 0. and local_pos_data.pose.position.y != 0.:
            X[0] = local_pos_data.pose.position.x
            X[1] = local_pos_data.pose.position.y
            X[2] = local_pos_data.pose.position.z
            #
            is_local_pos_data_ready = False
    elif is_vicon_data_ready:
        X[0] = vicon_data.pose.position.x
        X[1] = vicon_data.pose.position.y
        X[2] = vicon_data.pose.position.z
        #
        is_vicon_data_ready = False
    else:
        X[0] = x_initial
        X[1] = y_initial
        X[2] = 0.

    print(Style.BRIGHT + Fore.GREEN + "[ekf2] Initial Params: " + str(X) + Style.RESET_ALL)

    '''
    histroial parameters
    '''
    X_list = []
    t_X_list = []
    #
    P_list = []
    t_P_list = []
    #
    K_rpy_list = []
    t_K_rpy_list = []
    #
    K_flow_list = []
    t_K_flow_list = []
    #
    K_slam_list = []
    t_K_slam_list = []
    #
    K_servo_list = []
    t_K_servo_list = []

    '''
    process parameters
    '''
    index = 0
    t_attitude = time.time()
    t_attitude_last = time.time()
    #
    is_pos_optimized = False
    is_dist_updated = [ False for i in range(0, dist_array_num) ]
    dist_matrix   = np.zeros([dist_array_num, dist_array_num])
    dist_mat_last = np.zeros([dist_array_num, dist_array_num])                            # [swarm_num + 1, swarm_num + 1]    
    #
    X_opt_list = []
    X_opt_list_2 = []
    optimized_pose_last = []                                                    # (np.vstack([zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)]), [-1, -1, -1]), 数据结构, [(optimized_pose, [用于计算的三个id])]
    companion_pos_list = [[0, 0, 0] for i in range(0, dist_array_num)]          # 这个数据格式就是uav 1 - 9用于计算的位置, n * 3的矩阵
    update_id_pair = []
    #
    t_uwb = time.time()
    t_uwb_stage = t_uwb % (t_uwb_sampling + t_uwb_recv_cutoff)
    #
    x_local = np.zeros([3, 3])
    center_pos = np.zeros(3)
    is_local_pos_updated = False
    #
    while rclpy.ok():

        #
        # 打印要放在最前面
        # 要不然is_..._ready复位了
        if is_imu_data_ready and is_servo_data_ready:
        #if True:
            #
            # 在is_imu_data_ready是为了同步打印
            if is_debugging_ekf:
                if is_vicon_data_ready and vicon_data.pose.position != 0. and vicon_data.pose.position.y != 0:
                    print("[vicon data] x: %f, y: %f z: %f" % (vicon_data.pose.position.x, vicon_data.pose.position.y, vicon_data.pose.position.z))
                if is_gps_data_ready   and gps_data.pose.position != 0.   and gps_data.pose.position.y != 0:
                    print("[gps data]   x: %f, y: %f z: %f" % (gps_data.pose.position.x,   gps_data.pose.position.y,   gps_data.pose.position.z))
                #
                # print("X = [x, y, z, vx, vy, vz, q0, q1, q2, q3]")
                # print(X)
                # print(" ")

                #
                rpy_ekf_4_debugging = quaternion2euler([X[6], X[7], X[8], X[9]], is_degree=True)
                print("[EKF] rpy = %f %f %f" % (rpy_ekf_4_debugging[0], rpy_ekf_4_debugging[1], rpy_ekf_4_debugging[2],)) 
                #
                print("[EKF] vel     = %f %f %f" % (X[3], X[4], X[5],)) 
                print("[FCU] vel     = %f %f %f" % (vel_data.twist.linear.x, vel_data.twist.linear.y, vel_data.twist.linear.z,))
                print("[EKF] delta V = %f %f %f" % (X[3] - vel_data.twist.linear.x, X[4]- vel_data.twist.linear.y, X[5] - vel_data.twist.linear.z,))

        if is_imu_data_ready:
            #
            dt_imu = t_imu - t_imu_last
            if dt_imu > 1.:
                print("[EKF prediction] timing gap too large: %f, aborting once..." % (dt_imu, ))
                is_imu_data_ready = False
                continue
            # 注意顺序:
            # q = w  + xi  + yj  + zk
            #   = q0 + q1i + q2j + q3k
            quat      = [imu_data.orientation.w,      imu_data.orientation.x,      imu_data.orientation.y,      imu_data.orientation.z]
            quat_last = [imu_data_last.orientation.w, imu_data_last.orientation.x, imu_data_last.orientation.y, imu_data_last.orientation.z]
            #
            # prediction
            #
            acc_x_t = imu_data.linear_acceleration.x            # 方向修正
            acc_y_t = imu_data.linear_acceleration.y
            acc_z_t = imu_data.linear_acceleration.z
            acc_x_last = imu_data_last.linear_acceleration.x
            acc_y_last = imu_data_last.linear_acceleration.y
            acc_z_last = imu_data_last.linear_acceleration.z
            acc_x_t, acc_y_t, acc_z_t = acc_filter(acc_x_t, acc_y_t, acc_z_t, dt=dt_imu)
            #
            gyro_x_t =  imu_data.angular_velocity.x             # 方向修正
            gyro_y_t =  imu_data.angular_velocity.y             # TESTED
            gyro_z_t =  imu_data.angular_velocity.z             # 这里这么做是因为scipy的rpy旋转有点问题, 通过这个地方来对正, 需要注意的是Xk中的rpy定义是没问题的, 所以只改这边就行
                                                                 # 上真机的话可以通过只调整这个地方的正负以及rpy顺序来实现对齐
            gyro_x_last =  imu_data_last.angular_velocity.x
            gyro_y_last =  imu_data_last.angular_velocity.y     # TESTED, 第一次测试的时候归零初值关闭其他更新进行检查,做角运动, 但是尽量不要转动飞机, 防止旋转造成DCM改变
            gyro_z_last =  imu_data_last.angular_velocity.z
            #
            X, P = EKF_prediction(X, P, [acc_x_t, acc_y_t, acc_z_t],
                                        [acc_x_last, acc_y_last, acc_z_last],
                                        [gyro_x_t, gyro_y_t, gyro_z_t],
                                        [gyro_x_last, gyro_y_last, gyro_z_last], dt_imu, is_debugging=is_debugging_ekf)
            
            #
            # recording
            # X_list.append(X)
            # t_X_list.append(time.time())
            # P_list.append(P)
            # t_P_list.append(time.time())

            # if is_debugging_ekf:
            #     print("[FCU prediction] X = ")
            #     print(X)

            if index % 16 == 0:
            #if False:
                #
                t_attitude_last = float(t_attitude)
                t_attitude = time.time()
                dt_att = t_attitude - t_attitude_last
                if dt_att > 1.:
                    print("[FCU attitude] timing gap too large: %f, aborting once..." % (dt_att, ))     # 要是不加这个, 第一把的时候dt很大(因为t_last = 0), 就会把位置加到1e7左右
                    continue
                #
                rpy      = quaternion2euler(quat,      is_degree=False)
                rpy_last = quaternion2euler(quat_last, is_degree=False)            
                #X, K_rpy, P = EKF_update_attitude(X, P, rpy, rpy_last, None, None, dt_att, is_debugging=is_debugging_ekf)
                X, K_rpy, P = EKF_update_attitude(X, P, None, None, quat, quat_last, dt_att, is_debugging=is_debugging_ekf)

                # 
                X_list.append(X)
                t_X_list.append(time.time())
                P_list.append(P)
                t_P_list.append(time.time())
                # 
                K_rpy_list.append(K_rpy)
                t_K_rpy_list.append(t_attitude)


            if is_debugging_ekf:
            #if False:
                # rpy_fcu = quaternion2euler(quat, is_degree=True)
                # print("[rpy fcu] = ")
                # print(rpy_fcu)
                #
                # if is_debugging_ekf:
                #     print("[FCU attitude] X = ")
                #     print(X)
                pass

        if is_vel_data_ready:
        #if False:
            #
            dt_vel = t_vel - t_vel_last
            if dt_vel > 5.:
                print("[FCU velocity] timing gap too large: %f, aborting once..." % (dt_vel, ))
                is_vel_data_ready = False
                continue
            #
            vel_x = vel_data.twist.linear.x
            vel_y = vel_data.twist.linear.y             # TODO check orientation
            vel_z = vel_data.twist.linear.z
            #
            vel_x_last = vel_data_last.twist.linear.x
            vel_y_last = vel_data_last.twist.linear.y
            vel_z_last = vel_data_last.twist.linear.z
            #
            X, K_flow, P = EKF_update_fcu_velocity(X, P, [vel_x, vel_y, vel_z], [vel_x_last, vel_y_last, vel_z_last], dt_vel, is_debugging=is_debugging_ekf)

            # 
            X_list.append(X)
            t_X_list.append(time.time())
            P_list.append(P)
            t_P_list.append(time.time())
            # 
            K_flow_list.append(K_flow)
            t_K_flow_list.append(t_vel)

            # if is_debugging_ekf:
            #     print("[FCU velocity] X = ")
            #     print(X)            

        is_slam_data_ready = False
        if is_slam_data_ready:
            # TODO
            '''
            SLAM数据
            '''
            #X, K_slam, P = EKF_update_SLAM...
            pass

        if is_servo_data_ready and is_imu_data_ready and servo_data.__len__() > 0 and servo_data_last.__len__() > 0:
        #if False:
            #
            dt_servo = t_servo - t_servo_last    
            if dt_servo > 10.:
                print("[FCU servo] timing gap too large: %f, aborting once..." % (dt_servo, ))
                is_servo_data_ready = False
                continue
            #
            rpy = quaternion2euler(quat, is_degree=False)                       # TODO 注意这里要用到imu数据
            rol = rpy[0]
            pit = rpy[1]
            yaw = rpy[2]                            # TODO check orientation
            yaw = constraint_yaw(yaw, is_degree=False)
            X, K_servo, P = EKF_update_servo(X, P, servo_data, servo_data_last, dt_servo, [rol, pit, yaw], is_debugging=is_debugging_ekf)
    
            # 
            X_list.append(X)
            t_X_list.append(time.time())
            P_list.append(P)
            t_P_list.append(time.time())
            # 
            K_servo_list.append(K_servo)
            t_K_servo_list.append(t_servo)

            if is_debugging_ekf:
                print("[FCU servo] X = ")
                print(X)

        #
        # 速度位置LPF
        x_lpf,  y_lpf,  z_lpf  = pos_filter(X[0], X[1], X[2], dt=dt)
        vx_lpf, vy_lpf, vz_lpf = vel_filter(X[3], X[4], X[5], dt=dt)

        if is_debugging_ekf:
            print(Style.BRIGHT + Fore.GREEN + "[ekf2] optained pos: %f %f %f" % (x_lpf,  y_lpf,  z_lpf,  ) + Style.RESET_ALL)
            print(Style.BRIGHT + Fore.CYAN  + "[ekf2] optained vel: %f %f %f" % (vx_lpf, vy_lpf, vz_lpf, ) + Style.RESET_ALL)

        #
        # update UWB
        t_uwb = time.time() - t0
        t_uwb_stage = t_uwb % (t_uwb_sampling + t_uwb_recv_cutoff)

        if t_uwb_stage >= t_uwb_sampling and t_uwb_stage <= t_uwb_sampling + t_uwb_recv_cutoff:
            #
            # 收集数据
            #if is_debugging_uwb:
            if False:
                print(Fore.LIGHTBLUE_EX + "[ekf2 uwb] collecting data ..." + Style.RESET_ALL)


            # CRITICAL
            # reset
            is_pos_optimized = False
            is_last_pos_matched = False
            
            if is_this_uav_uwb_updated and is_comm_info_updated:
                #
                for i in range(0, dist_array_num):
                    companion_id_t = comm_info.uav[i].id
                    dist_list_t = comm_info.uav[i].dist.data
                    pos_t = [comm_info.uav[i].pos.pose.position.x, comm_info.uav[i].pos.pose.position.y, comm_info.uav[i].pos.pose.position.z]

                    #if is_debugging_uwb:
                    if False:
                        print(Fore.LIGHTBLUE_EX + "[ekf2 uwb] recv uav pos, id: %d" % (companion_id_t, ))
                        print(pos_t)
                        print(Style.RESET_ALL)

                    for j in range(0, dist_array_num):
                        if dist_list_t[j] != 0:
                            #
                            # 如果有数则更新取均值
                            # 否则直接直接更新
                            if dist_matrix[companion_id_t][j] == 0:
                                dist_matrix[companion_id_t][j] = dist_list_t[j]             # 其实仿真数据会多一些, 这里只有这么多数据了
                            else:
                                dist_matrix[companion_id_t][j] = (dist_list_t[j] + dist_matrix[companion_id_t][j]) / 2
                            #
                            # 这里用了个trick
                            # 一般uwb更新了, 位置大概率是更新的
                            is_dist_updated[companion_id_t] = True
                    #
                    # 更新位置
                    # 同样如果有数则更新取均值
                    # 否则直接直接更新
                    if not (pos_t[0] == 0 and pos_t[1] == 0 and pos_t[2] == 0):
                        if pos_t[0] != 0:
                            # x
                            if companion_pos_list[companion_id_t][0] == 0:                  # 注意, index = i不等于id = i
                                companion_pos_list[companion_id_t][0] = pos_t[0]
                            else:
                                companion_pos_list[companion_id_t][0] = (companion_pos_list[companion_id_t][0] + pos_t[0]) / 2
                        if pos_t[1] != 0:
                            # y
                            if companion_pos_list[companion_id_t][1] == 0:
                                companion_pos_list[companion_id_t][1] = pos_t[1]
                            else:
                                companion_pos_list[companion_id_t][1] = (companion_pos_list[companion_id_t][1] + pos_t[1]) / 2
                        if pos_t[2] != 0:
                            # z
                            if companion_pos_list[companion_id_t][2] == 0:
                                companion_pos_list[companion_id_t][2] = pos_t[2]
                            else:
                                companion_pos_list[companion_id_t][2] = (companion_pos_list[companion_id_t][2] + pos_t[2]) / 2

                #
                # finally
                # generate state update pair
                for i in range(0, dist_array_num):
                    for j in range(i + 1, dist_array_num):
                        if is_dist_updated[i] and is_dist_updated[j] and i != uav_id and j != uav_id:
                            group_id_i = comm_info.uav[i].group_id
                            group_id_j = comm_info.uav[j].group_id
                            #
                            # TODO
                            # Added, 为了方便计算, 可以只用一个group的数据
                            #if group_id == group_id_i and group_id == group_id_j:
                            if True:
                                update_id_pair.append((i, j, ))
                            else:
                                print(Style.BRIGHT + Fore.LIGHTYELLOW_EX + "[ekf2 uwb] WARNING find available data but not in THE SAME GROUP: %d %d %d" % (group_id, group_id_i, group_id_j, ) + Style.RESET_ALL)
                update_id_pair = list(set(update_id_pair))

            if is_this_uav_uwb_updated:
                is_this_uav_uwb_updated = False
            if is_comm_info_updated:
                is_comm_info_updated = False

            #if is_debugging_uwb:
            if False:
                print(Fore.LIGHTCYAN_EX + "[ekf2 uwb] dist mat")
                print(dist_matrix)
                print(Style.RESET_ALL)

        try:
        #if True:                                                # TODO Critical
            #
            # 这里因为dt很小所以 * 10.1
            # 否则乘以5.1甚至2.1就够了
            if not is_pos_optimized and t_uwb_stage >= t_uwb_sampling + t_uwb_recv_cutoff - dt * 10.1 and t_uwb_stage <= t_uwb_sampling + t_uwb_recv_cutoff - dt * 1.1:
                # if is_debugging_uwb:
                if False:
                    print(Style.BRIGHT + Fore.LIGHTMAGENTA_EX + "[ekf2 uwb] calculating data ..., t_all, t_stage %f %f" % (t_uwb, t_uwb_stage, ) + Style.RESET_ALL)
                    #
                    print(Fore.LIGHTMAGENTA_EX + "[ekf2 uwb] dist mat")
                    print(dist_matrix)
                    print(Style.RESET_ALL)
                    #
                    print(Fore.MAGENTA + "[ekf2 uwb] companion pos")
                    print(companion_pos_list)
                    print(Style.RESET_ALL)
                    #
                    #
                    print(Fore.MAGENTA + "[ekf2 uwb] update_id_pair")
                    print(update_id_pair)
                    print(Style.RESET_ALL)               

                #
                # 总体位置优化
                for k in range(0, update_id_pair.__len__()):
                    i = update_id_pair[k][0]
                    j = update_id_pair[k][1]
                    #
                    pos_uav_i = companion_pos_list[i]
                    pos_uav_j = companion_pos_list[j]
                    #
                    #X_opt_t = np.vstack([zeros(3), zeros(3), [x_lpf, y_lpf, z_lpf], pos_uav_i, pos_uav_j])
                    X_opt_t = np.zeros([dist_array_num, 3])
                    X_opt_t[i][0]      = pos_uav_i[0]; X_opt_t[i][1]      = pos_uav_i[1]; X_opt_t[i][2]      = pos_uav_i[2]
                    X_opt_t[j][0]      = pos_uav_j[0]; X_opt_t[j][1]      = pos_uav_j[1]; X_opt_t[j][2]      = pos_uav_j[2]
                    X_opt_t[uav_id][0] = x_lpf;        X_opt_t[uav_id][1] = y_lpf;        X_opt_t[uav_id][2] = z_lpf                #
                    #
                    if is_debugging_uwb:
                    #if False:
                        # TODO
                        # TO check out value
                        print(Fore.LIGHTYELLOW_EX + "[ekf2 uwb] opti_IN_frames X")
                        print(X_opt_t)
                        print("[ekf2 uwb] opti_BETWEEN_frames dist_mat")
                        print(dist_matrix)
                        print("[ekf2 uwb] opti_BETWEEN_frames uavs: %d %d %d", (uav_id, i, j,))
                        print(Style.RESET_ALL)
                    #
                    X_opt_t = optimize_pose_in_frames(X_opt_t, dist_matrix, [uav_id, i, j])
                    X_opt_list.append(X_opt_t)

                    #
                    #if is_debugging_uwb:
                    if False:
                        print("[ekf2 uwb] optimized_pose_last")
                        print(optimized_pose_last)

                    #
                    is_last_pos_matched = False
                    matched_opti_pose_index = -1
                    if optimized_pose_last.__len__() and update_id_pair.__len__():                               # 第一帧不计算
                        for p in range(0, optimized_pose_last.__len__()):
                            id_list_last = optimized_pose_last[p][1]
                            is_all_id_matched = True
                            for q in range(0, 3):
                                current_id_t = id_list_last[q]
                                if not (current_id_t == uav_id or current_id_t in update_id_pair[k]):           # 所有ID都必须对上
                                    is_all_id_matched = False                                                   # 这里顺序应该没关系
                                    break
                            if is_all_id_matched:
                                matched_opti_pose_index = p
                                is_last_pos_matched = True

                                if is_debugging_uwb:
                                    print(Fore.MAGENTA + "[ekf2 uwb] found_last pos for uavs: %d %d %d" % (uav_id, update_id_pair[k][0], update_id_pair[k][1]))
                                    print(optimized_pose_last[matched_opti_pose_index][0])
                                    print(Style.RESET_ALL)

                                break
                    #
                    if is_last_pos_matched:
                        #
                        # X_opt_2数据结构
                        # [[0, 0, 0],
                        #  [0, 0, 0],
                        #  [x_t, y_t, z_t], # 本机
                        #  [x_i, y_i, z_i], # uav i
                        #  [x_j, y_j, z_j], # uav j
                        #  ]
                        if is_debugging_uwb:
                            print("[ekf2 uwb] opti_BETWEEN_frames X_last")
                            print(optimized_pose_last[k][0])
                            print("[ekf2 uwb] opti_BETWEEN_frames X")
                            print(X_opt_t)
                            # print("[ekf2 uwb] opti_BETWEEN_frames dist_mat_last")
                            # print(dist_mat_last)
                            # print("[ekf2 uwb] opti_BETWEEN_frames dist_mat")
                            # print(dist_matrix)

                        X_opt_t_2 = optimize_pose_between_frames(optimized_pose_last[matched_opti_pose_index][0], X_opt_t, dist_mat_last, dist_matrix, [uav_id, i, j])
                        X_opt_list_2.append(X_opt_t_2)

                # Finally
                # 取平均值
                # 一定要在最后取, 避免之前结果拉扯后续结果
                if is_last_pos_matched and X_opt_list_2.__len__():
                    #
                    #if is_debugging_uwb:
                    if False:
                        print("[ekf2 uwb] X_opt_list_2")
                        print(X_opt_list_2)
                    #
                    for p in range(0, X_opt_list_2.__len__()):
                        x_lpf = x_lpf + (X_opt_list_2[p][uav_id][0] - x_lpf) / X_opt_list_2.__len__()           # TODO, to checkout
                        y_lpf = y_lpf + (X_opt_list_2[p][uav_id][1] - y_lpf) / X_opt_list_2.__len__()           # x + dx的平均值
                        z_lpf = z_lpf + (X_opt_list_2[p][uav_id][2] - z_lpf) / X_opt_list_2.__len__()           # 最后统一改到位置输出内, 这个数据不能并入EKF否则效果会很差

                    print(Style.BRIGHT + Fore.GREEN + "[ekf2 uwb] BETWEEN Frames X: %f %f %f" % (x_lpf, y_lpf, z_lpf, ) + Style.RESET_ALL)

                elif not is_last_pos_matched and X_opt_list.__len__():
                    for p in range(0, X_opt_list.__len__()):
                        x_lpf = x_lpf + (X_opt_list[p][uav_id][0] - x_lpf) / X_opt_list.__len__()
                        y_lpf = y_lpf + (X_opt_list[p][uav_id][1] - y_lpf) / X_opt_list.__len__()
                        z_lpf = z_lpf + (X_opt_list[p][uav_id][2] - z_lpf) / X_opt_list.__len__()

                    print(Style.BRIGHT + Fore.GREEN + "[ekf2 uwb] IN Frames X: %f %f %f" % (x_lpf, y_lpf, z_lpf, ) + Style.RESET_ALL)
                else:
                    print(Style.BRIGHT + Fore.YELLOW + "[ekf2 uwb] NO Solution this time" + Style.RESET_ALL)
                    pass

                # TODO
                # 计算局部位置
                # 注意这个只在本组内计算
                for k in range(0, update_id_pair.__len__()):
                    #
                    # step 1 check is in the same swarm group
                    id_i_t = update_id_pair[k][0]
                    id_j_t = update_id_pair[k][1]
                    is_group_id_i_matched = False
                    is_group_id_j_matched = False
                    for p in range(0, comm_info.uav.__len__()):
                        if id_i_t == comm_info.uav[p].id and group_id == comm_info.uav[p].group_id:
                            is_group_id_i_matched = True
                        if id_j_t == comm_info.uav[p].id and group_id == comm_info.uav[p].group_id:
                            is_group_id_j_matched = True

                    #
                    # step 2
                    # check whethet last pos matched
                    is_last_pos_matched = False
                    matched_opti_pose_index = -1
                    if optimized_pose_last.__len__() and update_id_pair.__len__():                               # 第一帧不计算
                        for p in range(0, optimized_pose_last.__len__()):
                            id_list_last = optimized_pose_last[p][1]
                            is_all_id_matched = True
                            for q in range(0, 3):
                                current_id_t = id_list_last[q]
                                if not (current_id_t == uav_id or current_id_t in update_id_pair[k]):           # 所有ID都必须对上
                                    is_all_id_matched = False                                                   # 这里顺序应该没关系
                                    break
                            if is_all_id_matched:
                                matched_opti_pose_index = p
                                is_last_pos_matched = True

                                if is_debugging_uwb:
                                    print(Fore.MAGENTA + "[ekf2 uwb] local found_last pos for uavs: %d %d %d" % (uav_id, update_id_pair[k][0], update_id_pair[k][1]))
                                    print(optimized_pose_last[matched_opti_pose_index][0])
                                    print(Style.RESET_ALL)

                                break


                    #
                    # step 3 updated
                    #if is_group_id_i_matched and is_group_id_j_matched and is_last_pos_matched:            # TODO 需要同组计算
                    if is_last_pos_matched:
                        center_pos, x_local = local_pos_alginment(np.array([X_opt_t_2[uav_id], X_opt_t_2[id_i_t], X_opt_t_2[id_j_t]]),
                                                                    np.array([optimized_pose_last[matched_opti_pose_index][0][uav_id], optimized_pose_last[matched_opti_pose_index][0][id_i_t], optimized_pose_last[matched_opti_pose_index][0][id_j_t]]),
                                                                    dist_mat_last,
                                                                    dist_matrix,
                                                                    [uav_id, id_i_t, id_j_t])
                        print(Style.BRIGHT + Fore.GREEN + "[ekf2 uwb] LOCAL X: %f %f %f, center: %f %f %f" % (x_local[uav_id][0], x_local[uav_id][1], x_local[uav_id][2], center_pos[0], center_pos[1], center_pos[2], ) + Style.RESET_ALL)
                        is_local_pos_updated = True

                #
                # iteration
                optimized_pose_last.clear()         #For clearing, optimized_pose_last = np.vstack([zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)])
                for k in range(0, update_id_pair.__len__()):
                    #
                    #optimized_pose_last_t = np.vstack([zeros(3), zeros(3), [x_lpf, y_lpf, z_lpf], pos_uav_i, pos_uav_j])
                    #
                    i = update_id_pair[k][0]
                    j = update_id_pair[k][1]
                    if X_opt_list_2.__len__():
                        pos_uav_i = X_opt_list_2[k][i]
                        pos_uav_j = X_opt_list_2[k][j]
                        pos_uav_t = X_opt_list_2[k][uav_id]
                    elif X_opt_list.__len__():
                        pos_uav_i = X_opt_list[k][i]
                        pos_uav_j = X_opt_list[k][j]
                        pos_uav_t = X_opt_list[k][uav_id]
                    else:
                        pos_uav_i = np.array([comm_info.uav[i].pos.pose.position.x, comm_info.uav[i].pos.pose.position.y, comm_info.uav[i].pos.pose.position.z])
                        pos_uav_j = np.array([comm_info.uav[j].pos.pose.position.x, comm_info.uav[j].pos.pose.position.y, comm_info.uav[j].pos.pose.position.z])
                        pos_uav_t = np.array([x_lpf, y_lpf, z_lpf])

                    optimized_pose_last_t = np.zeros([dist_array_num, 3])
                    optimized_pose_last_t[i][0]      = pos_uav_i[0]; optimized_pose_last_t[i][1]      = pos_uav_i[1]; optimized_pose_last_t[i][2]      = pos_uav_i[2]
                    optimized_pose_last_t[j][0]      = pos_uav_j[0]; optimized_pose_last_t[j][1]      = pos_uav_j[1]; optimized_pose_last_t[j][2]      = pos_uav_j[2]
                    optimized_pose_last_t[uav_id][0] = pos_uav_t[0]; optimized_pose_last_t[uav_id][1] = pos_uav_t[1]; optimized_pose_last_t[uav_id][2] = pos_uav_t[2]
                    
                    optimized_pose_last.append((optimized_pose_last_t, (uav_id, i , j)))
                #
                for p in range(0, dist_matrix.shape[0]):
                    for q in range(0, dist_matrix.shape[1]):
                        dist_mat_last[p][q] = dist_matrix[p][q]
                
                #if is_debugging_uwb:
                if False:
                    print("[ekf2 uwb] updated optimized pose last")
                    print(optimized_pose_last)
                    #
                    print("[ekf2 uwb] updated dist_mat_last")
                    print(dist_mat_last)

                # TODO
                # 完成计算后清零dist martix
                is_pos_optimized = True
                #
                is_dist_updated.clear()
                is_dist_updated = [ False for i in range(0, dist_array_num) ]
                #
                dist_matrix = np.zeros([10, 10])
                #
                companion_pos_list.clear()
                companion_pos_list = [ [0, 0, 0] for i in range(0, dist_array_num) ]
                #
                update_id_pair.clear()
        #
        # TODO CRITICAL
        except IndexError as e:
            print(e)
            print(Style.BRIGHT + Fore.LIGHTRED_EX + "[ekf2 uwb] WARNING EKF2 UWB Error" + Style.RESET_ALL)
        except KeyError as e:
            print(e)
            print(Style.BRIGHT + Fore.LIGHTRED_EX + "[ekf2 uwb] WARNING EKF2 UWB Error" + Style.RESET_ALL)
        except ValueError as e:
            print(e)
            print(Style.BRIGHT + Fore.LIGHTRED_EX + "[ekf2 uwb] WARNING EKF2 UWB Error" + Style.RESET_ALL)
        except Exception as e:
            print(e)
            print(Style.BRIGHT + Fore.LIGHTRED_EX + "[ekf2 uwb] WARNING EKF2 UWB Error" + Style.RESET_ALL)

        #
        # TODO
        # publishing data

        if is_vel_data_ready:
            ekf2_pose.header.stamp = node.get_clock().now().to_msg()
            ekf2_pose.header.frame_id = 'base_link'
            #
            ekf2_pose.pose.position.x = x_lpf           # X[0]
            ekf2_pose.pose.position.y = y_lpf           # X[1]
            ekf2_pose.pose.position.z = z_lpf           # X[2]
            #
            ekf2_pose.pose.orientation.w = X[6]
            ekf2_pose.pose.orientation.x = X[7]
            ekf2_pose.pose.orientation.y = X[8]
            ekf2_pose.pose.orientation.z = X[9]
            #
            ekf2_pos_pub.publish(ekf2_pose)

            # print(Style.BRIGHT + Fore.GREEN + "[ekf2] published pos: %f %f %f" % (ekf2_pose.pose.position.x,  ekf2_pose.pose.position.y,  ekf2_pose.pose.position.z,  ) + Style.RESET_ALL)            

            #
            ekf2_vel.header.stamp = node.get_clock().now().to_msg()
            ekf2_vel.header.frame_id = 'base_link'
            ekf2_vel.twist.linear.x = vx_lpf            # X[3]
            ekf2_vel.twist.linear.y = vy_lpf            # X[4]
            ekf2_vel.twist.linear.z = vz_lpf            # X[5]
            ekf2_vel.twist.angular.x = imu_data.angular_velocity.x
            ekf2_vel.twist.angular.y = imu_data.angular_velocity.y
            ekf2_vel.twist.angular.z = imu_data.angular_velocity.z
            #
            ekf2_vel_pub.publish(ekf2_vel)

            # print(Style.BRIGHT + Fore.CYAN + "[ekf2] published vel: %f %f %f" % (ekf2_vel.twist.linear.x, ekf2_vel.twist.linear.y, ekf2_vel.twist.linear.z, ) + Style.RESET_ALL)

            # TODO
            # 发布局部位置
            if is_local_pos_updated:
                ekf2_uwb_local_pos.header.stamp = node.get_clock().now().to_msg()                
                ekf2_uwb_local_pos.header.frame_id = 'swarm_center_ned'
                ekf2_uwb_local_pos.pose.position.x = x_local[uav_id][0]
                ekf2_uwb_local_pos.pose.position.y = x_local[uav_id][1]
                ekf2_uwb_local_pos.pose.position.z = x_local[uav_id][2]
                #
                ekf2_uwb_local_pos_pub.publish(ekf2_uwb_local_pos)

                ekf2_uwb_center_pos.header.stamp = node.get_clock().now().to_msg()                
                ekf2_uwb_center_pos.header.frame_id = 'map'
                ekf2_uwb_center_pos.pose.position.x = center_pos[0]
                ekf2_uwb_center_pos.pose.position.y = center_pos[1]
                ekf2_uwb_center_pos.pose.position.z = center_pos[2]
                #
                ekf2_uwb_center_pos_pub.publish(ekf2_uwb_center_pos)


        if is_imu_data_ready:
            is_imu_data_ready   = False
        if is_vel_data_ready:
            is_vel_data_ready   = False
        if is_slam_data_ready:
            is_slam_data_ready  = False
        if is_servo_data_ready:
            is_servo_data_ready = False

        #
        # counter
        index = index + 1
        if index % 1000:
            index = 0
        #
        time.sleep(dt)
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
