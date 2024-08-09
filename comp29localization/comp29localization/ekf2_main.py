# -*- coding: UTF-8 -*-
#!/usr/bin/python3

import rclpy
import time
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import numpy as np
import scipy
from numpy import array

from ekf_utils import dshot_2_T
from ekf_utils import quaternion2euler, euler2quaternion, constraint_yaw, constraint_float

'''
    UAV Parameters
'''
uav_id  = 3
acc_max = 5  # m / s^2
uav_m = 1.35
g = 9.81
'''
    Debugging options
'''
is_debugging_data_recv = False
is_debugging_ekf = True
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

def main(args=None):
    #
    global imu_data,       imu_data_last,       t_imu,       t_imu_last,       is_imu_data_ready
    global servo_data,     servo_data_last,     t_servo,     t_servo_last,     is_servo_data_ready
    global vel_data,       vel_data_last,       t_vel,       t_vel_last,       is_vel_data_ready
    global local_pos_data, local_pos_data_last, t_local_pos, t_local_pos_last, is_local_pos_data_ready
    global gps_data,       gps_data_last,       t_gps,       t_gps_last,       is_gps_data_ready
    global vicon_data,     vicon_data_last,     t_vicon,     t_vicon_last,     is_vicon_data_ready

    rclpy.init(args=args)
    node = Node('data_collector')

    #
    imu_sub             = node.create_subscription(Imu, 'imu', imu_cb, 10)
    servo_sub           = node.create_subscription(Float32MultiArray, 'servo_data', servo_cb, 1)
    velocity_sub        = node.create_subscription(TwistStamped, 'velocity_and_angular', velocity_cb, 1)
    #
    local_pos_sub       = node.create_subscription(PoseStamped, 'local_position_ned', local_pos_cb, 1)
    gps_pos_sub         = node.create_subscription(PoseStamped, 'gps_position', gps_cb, 1)
    vicon_pos_sub       = node.create_subscription(PoseStamped, '/vicon_pose', vicon_cb, 1)
    #
    ekf2_pos_topic_name = '/uav' + str(uav_id) + "/ekf2/pose"
    ekf2_pos_pub        = node.create_publisher(PoseStamped,  ekf2_pos_topic_name, 1)
    ekf2_vel_topic_name = '/uav' + str(uav_id) + "/ekf2/vel"    
    ekf2_vel_pub        = node.create_publisher(TwistStamped, ekf2_vel_topic_name, 1)
    ekf2_pose = PoseStamped()
    ekf2_vel  = TwistStamped()

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
        print("[ekf2] waiting for imu data ...")
        time.sleep(0.5)
        rclpy.spin_once(node)

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
    if is_local_pos_data_ready:
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

        if is_servo_data_ready and servo_data.__len__() > 0 and servo_data_last.__len__() > 0:
        #if False:
            #
            dt_servo = t_servo - t_servo_last    
            if dt_servo > 10.:
                print("[FCU servo] timing gap too large: %f, aborting once..." % (dt_servo, ))
                is_servo_data_ready = False
                continue
            #
            rpy = quaternion2euler(quat, is_degree=False)
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
        # TODO
        # update UWB

        #
        # TODO
        # publishing data

        if is_vel_data_ready:
            ekf2_pose.header.stamp = node.get_clock().now().to_msg()
            ekf2_pose.header.frame_id = 'base_link'
            #
            ekf2_pose.pose.position.x = X[0]
            ekf2_pose.pose.position.y = X[1]
            ekf2_pose.pose.position.z = X[2]
            #
            ekf2_pose.pose.orientation.w = X[6]
            ekf2_pose.pose.orientation.x = X[7]
            ekf2_pose.pose.orientation.y = X[8]
            ekf2_pose.pose.orientation.z = X[9]
            #
            ekf2_pos_pub.publish(ekf2_pose)

            #
            ekf2_vel.header.stamp = node.get_clock().now().to_msg()
            ekf2_vel.header.frame_id = 'base_link'
            ekf2_vel.twist.linear.x = X[3]
            ekf2_vel.twist.linear.y = X[4]
            ekf2_vel.twist.linear.z = X[5]
            ekf2_vel.twist.angular.x = imu_data.angular_velocity.x
            ekf2_vel.twist.angular.y = imu_data.angular_velocity.y
            ekf2_vel.twist.angular.z = imu_data.angular_velocity.z
            #
            ekf2_vel_pub.publish(ekf2_vel)

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
        time.sleep(0.001)
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
