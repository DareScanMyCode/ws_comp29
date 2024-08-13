from numpy import *
# from rosbags.highlevel import AnyReader
# from rosbags.rosbag1 import Reader as BagReader
# from rosbags.typesys.types import sensor_msgs__msg__Imu as Imu
# import rosbag
import scipy
from scipy.optimize import minimize
# from pyulog import ULog
import pandas as pd
import numpy as np
import re
from scipy.spatial.transform import Rotation as R
from scipy.integrate import simpson, romberg, trapezoid, cumulative_trapezoid
# import matplotlib
# import matplotlib.pyplot as plt
import bisect
# matplotlib.use('TkAgg')  # 使用Tkinter后端

#
kt  = 1e-6 # 转速和升力关系系数
kr  = 8e-7 # 转速和力矩关系系数
# 拉力曲线参照 https://item.taobao.com/item.htm?spm=a21n57.1.item.4.2c191cc2HrWahR&priceTId=213e377f17214550475931805e1067&utparam=%7B%22aplus_abtest%22:%22d8071d5aba29ca7094776a086f465e22%22%7D&id=704202196861&ns=1&abbucket=5&sku_properties=1627207:23681135502
k_pacer3 = (1400 - 0) / (1 - 0.1) * 0.00981  # 0.00981 克力到牛顿 https://citizenmaths.com/zh-cn/force/grams-force-to-newtons
deadzone_pacer3 = 0.1  # 点斜表示 y - y0 = k * (x - x0), y0 = 0
#
t_m_ratio = 2.131e-8 / 2.120e-6

default_start_time   =  1721121093007830000
# default_start_time = 1721121088796820000

def extract_numbers(string):
    # 使用正则表达式提取所有数字
    numbers = re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', string)
    # 将提取的数字转换为整数列表
    numbers = [float(num) for num in numbers]
    return numbers

def imu_get(df, start_time=default_start_time):
    imu_array = {
        'timestamp': [], 'sec': [], 'nanosec': [], 'Quaternion': [], 'euler':[],'angular_velocity': [],
        'linear_acceleration': []}

    # 获取时间戳
    imu_timestamp = df['timestamp']
    for i in range(0,len(imu_timestamp)):
        if imu_timestamp[i] > start_time:
            imu_array['timestamp'].append((df['timestamp'][i] - start_time) / 1e9)
            imu_array['sec'].append(df['header.stamp.sec'][i])
            imu_array['nanosec'].append(df['header.stamp.nanosec'][i])
            imu_array['Quaternion'].append([df['orientation.x'][i],df['orientation.y'][i],df['orientation.z'][i],df['orientation.w'][i]])
            imu_array['angular_velocity'].append([df['angular_velocity.x'][i],df['angular_velocity.y'][i],df['angular_velocity.z'][i]])
            imu_array['linear_acceleration'].append([df['linear_acceleration.x'][i],df['linear_acceleration.y'][i],df['linear_acceleration.z'][i]])
            quat = [df['orientation.w'][i],df['orientation.x'][i],df['orientation.y'][i],df['orientation.z'][i]]
            r    = R.from_quat(quat)
            imu_array['euler'].append(list(r.as_euler('xyz',degrees=False)))

    return imu_array

def quaternion2euler(quaternion, is_degree=True):
    r = R.from_quat(quaternion)
    euler = r.as_euler('zyx', degrees=is_degree)                # 注意这个顺序
    return euler

def euler2quaternion(euler, is_degree=True):
    r = R.from_euler('zyx', euler, degrees=is_degree)
    quaternion = r.as_quat()
    return quaternion

def constraint_float(x, lower, upper):
    if x >= upper:
        return upper
    elif x <= lower:
        return lower
    else:
        return x

def constraint_yaw(x, is_degree=True):
    if is_degree:
        while x > 180.:
            x -= 360.
        while x <= -180.:
            x += 360.
    else:
        while x > np.pi:
            x -= np.pi * 2
        while x <= -np.pi:
            x += np.pi * 2
    return x

def servo_data_get(df, start_time=default_start_time):
    servo_data_msg = df['data']
    # 获取筛选后的数据中的 'message' 列

    servo_data_dict = {
        'timestamp': [],'motor1': [],'motor2': [],'motor3': [],'motor4': []
    }
    # 获取时间戳
    servo_timestamp = df['timestamp']
    for i in range(0,len(servo_timestamp)):
        if float(servo_timestamp[i]) > start_time:
            servo_data_dict['timestamp'].append((df['timestamp'][i] - start_time)  / 1e9)
            data = extract_numbers(servo_data_msg[i])
            # print(data)
            # input()
            # 添加数据到字典
            servo_data_dict['motor1'].append(data[-4])
            servo_data_dict['motor2'].append(data[-3])
            servo_data_dict['motor3'].append(data[-2])
            servo_data_dict['motor4'].append(data[-1])
    return servo_data_dict

def localposition_get(df):

    local_pos_list = {'timestamp':[],'sec': [], 'nanosec': [],'local_position': [],'Quaternion': []}
    #
    # # 获取时间戳
    # loc_pos_timestamp = local_position['timestamp']
    # for i in range(0,len(loc_pos_timestamp)):
    #     local_pos_list['sec'].append(data[0])
    #     local_pos_list['nanosec'].append(data[1])
    #     local_pos_list['local_position'].append([data[2], data[3], data[4]])
    #     local_pos_list['Quaternion'].append([data[5],data[6], data[7], data[8],])
    return local_pos_list

def v_w_get(df,imu_list, start_time=default_start_time):
    v_w_data_dict = {
        'timestamp': [], 'linear': [], 'angular': [], 'orientation':[]
    }
    timestamp = df['timestamp']
    close_indices = find_closest_indices(imu_list['timestamp'],timestamp)
    for i in range(0,len(timestamp)):
        if timestamp[i] > start_time:
            v_w_data_dict['timestamp'].append((df['timestamp'][i] - start_time)  / 1e9)
            v_w_data_dict['linear'].append([df['twist.linear.x'][i],df['twist.linear.y'][i],df['twist.linear.z'][i]])
            v_w_data_dict['angular'].append([df['twist.angular.x'][i],df['twist.angular.y'][i],df['twist.angular.z'][i]])
            v_w_data_dict['orientation'].append(imu_list['Quaternion'][close_indices[i]])
    return v_w_data_dict

def gps_get(df, start_time=default_start_time):
    gps_dict = {
        'timestamp': [], 'lat': [], 'lon': [], 'position': []
    }
    timestamp   = df['timestamp']
    lat_ref     = df['pose.position.x'][0]  #将第一个点作为起点
    lon_ref     = df['pose.position.y'][0]  #为了转化成位置计算
    for i in range(0, len(timestamp)):
        if timestamp[i]>start_time:
            gps_dict['timestamp'].append((timestamp[i]-start_time)/1e9)
            gps_dict['lat'].append(df['pose.position.x'][i]-lat_ref)
            gps_dict['lon'].append(df['pose.position.y'][i]-lon_ref)
            x = (df['pose.position.x'][i]-lat_ref) * 111000  # 纬度差转换为米
            y = (df['pose.position.y'][i]-lon_ref) * 111000 * cos(radians(lat_ref))  # 经度差转换为米
            gps_dict['position'].append([x,y])

    return gps_dict

def vicon_get(df, start_time=default_start_time):
    vicon_dict = {
        'timestamp':[],'pose':[],'orientation':[],'v_compute':[],'euler':[]
    }
    timestamp = df['timestamp']
    for i in range(0,len(timestamp)):
        if float(timestamp[i]) > start_time:
            vicon_dict['timestamp'].append(timestamp[i])
            vicon_dict['pose'].append([df['pose.position.x'][i],df['pose.position.y'][i],df['pose.position.z'][i]])

            qx = df['pose.orientation.x'][i]
            qy = df['pose.orientation.y'][i]
            qz = df['pose.orientation.z'][i]
            qw = df['pose.orientation.w'][i]
            vicon_dict['orientation'].append([qx, qy, qz, qw])
            # 取大致0.1s作为间隔
            vx = (df['pose.position.x'][i] - df['pose.position.x'][i - 10]) * 1e9 / float(timestamp[i] - timestamp[i - 10])
            vy = (df['pose.position.y'][i] - df['pose.position.y'][i - 10]) * 1e9 / float(timestamp[i] - timestamp[i - 10])
            vz = (df['pose.position.z'][i] - df['pose.position.z'][i - 10]) * 1e9 / float(timestamp[i] - timestamp[i - 10])
            # vx = (df['pose.position.x'][i+5] - df['pose.position.x'][i - 5]) * 1e9 / float(timestamp[i+5] - timestamp[i - 5])
            # vy = (df['pose.position.y'][i+5] - df['pose.position.y'][i - 5]) * 1e9 / float(timestamp[i+5] - timestamp[i - 5])
            # vz = (df['pose.position.z'][i+5] - df['pose.position.z'][i - 5]) * 1e9 / float(timestamp[i+5] - timestamp[i - 5])
            vicon_dict['v_compute'].append([vx,vy,vz])

            phi = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
            theta = np.arcsin(2 * (qw * qy - qz * qx))
            psi = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
            vicon_dict['euler'].append([phi,theta,psi])
    return vicon_dict

def uwb_get(df,start_time):
    """
    [function]： 将获得的距离字符串自动转换为n*n维度的矩阵
    [warn]：     如果前后差值大于前面的百分50则去除
    """
    uwb_data_dict = {
        'timestamp':[],'dist_matrix':[]
    }
    timestamp = df['timestamp']

    for i in range(0,len(timestamp)):
        if timestamp[i] > start_time:
            data = array(extract_numbers(df['data'][i]))
            n = int(np.sqrt(len(data)))  # 确定矩阵的维数 n
            percent_diff = zeros(n)
            if i != 0:
                data_1 = array(extract_numbers(df['data'][i - 1]))
                diff = abs(data - data_1)
                epsilon = 1e-6   # 避免除以零，添加一个小值
                percent_diff = diff / (data_1 + epsilon) # 计算差值占第一个数组值的百分比
            if abs(max(percent_diff)) < 0.5 or i == 0:
                uwb_data_dict['timestamp'].append((timestamp[i]-start_time) / 1e9)
                uwb_matrix = np.array(data).reshape(n, n)
                uwb_data_dict['dist_matrix'].append(uwb_matrix)
    return uwb_data_dict

def Vel_Prediction(X_k1, v, dt):
    # X = [x y z]'
    trans = transpose(array([
        v[0],
        v[1],
        v[2]
    ]))
    X_k = X_k1 + trans * dt

    return X_k

def Vel_only_integral(pos_uav, UAV_vw, record_vicon_uav,segment):
    for i in segment:
        if i == 0:
             dt = UAV_vw['timestamp'][i]
        else:
             dt = UAV_vw['timestamp'][i] - UAV_vw['timestamp'][i - 1]
        pos_uav = Vel_Prediction(pos_uav, UAV_vw['linear'][i], dt)
        record_vicon_uav = np.vstack([record_vicon_uav, pos_uav])
    return record_vicon_uav,pos_uav

def dshot_2_T(value):
    global k_pacer3, deadzone_pacer3

    T = k_pacer3 * ((float(value) - 48) / 2000. - deadzone_pacer3)
    if T < 0:
        T = 0  # 当前阶段是不可能产生负升力的, 下同
    return T

def dshot_2_M(value):
    global k_pacer3, deadzone_pacer3, t_m_ratio

    T = k_pacer3 * ((float(value) - 48) / 2000. - deadzone_pacer3)
    M = t_m_ratio * T
    if M < 0:
        M = 0
    return M

def get_d_horizon(Xt1, Xt, D):
    if -1 <= (Xt[2] - Xt1[2]) / D <= 1:
        theta = arcsin((Xt[2] - Xt1[2]) / D)
        distance = D * cos(theta)
        return distance
    else:
        print("[error function]: get_d_horizon [reason]: 反正弦值不在[-1 1]内，没办法投影到水平面，返回值为 -1 ！")
        return -1


''' 获取两个向量之间的夹角'''
def angle_between_vectors(a, b):
    # 计算向量 a 和 b 的点积
    dot_product = dot(a, b)
    # 计算向量 a 和 b 的欧几里得范数
    norm_a = linalg.norm(a)
    norm_b = linalg.norm(b)
    # 计算夹角的余弦值
    cos_theta = dot_product / (norm_a * norm_b)
    # 计算夹角（弧度）
    theta = arccos(clip(cos_theta, -1.0, 1.0))
    return theta


'''
    optimize_pose_between_frames
    解释：       帧间位姿优化 
    输入参数解释： Xt1,Xt,Vt  Xn1,Xn,Vn 均为列向量，参考以下格式：
                    Xt1: [ x_t-1 y_t-1 ]T  Vt: [ v_y v_z ]T
                Dt,Dn: 为UWB获得二者之间的距离 (不为水平投影！！！)
'''

def find_closest_indices(reference_timestamps, needed_timestamps):
    """
    找到每个 needed_timestamps 时间戳在 reference_timestamps 中最近的下标。
    返回:
        list: 每个 needed_timestamps 时间戳在 reference_timestamps 中最近的下标列表。
    """
    closest_indices = []

    for servo_time in needed_timestamps:
        # 找到比 servo_time 大的第一个 imu 时间戳的位置
        pos = bisect.bisect_right(reference_timestamps, servo_time)

        # 检查 pos 及其前一个位置，以找到最近的 imu 时间戳
        if pos == 0:
            closest_indices.append(0)
        elif pos == len(reference_timestamps):
            closest_indices.append(len(reference_timestamps) - 1)
        else:
            before = pos - 1
            after = pos
            # 找到最近的时间戳
            if abs(reference_timestamps[before] - servo_time) <= abs(reference_timestamps[after] - servo_time):
                closest_indices.append(before)
            else:
                closest_indices.append(after)

    return closest_indices

def compute_velocity_from_dict(data):
    velocities = []
    current_velocity = np.array([0.0, 0.0, 0.0])
    last_timestamp = None
    for i in range(0,len(data['timestamp'])):
        timestamp = data['timestamp'][i]
        if last_timestamp is not None:
            dt = timestamp - last_timestamp
            accel = np.array(data['linear_acceleration'][i])
            print(accel)
            # input()
            current_velocity += accel * dt

        velocities.append(current_velocity.copy())
        last_timestamp = timestamp

    return velocities

def convert_to_global(body_vector,Quaternion):
    """
    :param body_vector:     所需计算的机身的坐标系
    :param Quaternion:      当前时刻的四元数角
    :return: global_vector: 返回世界坐标系
    """
    r = R.from_quat(Quaternion)
    rotation_matrix = r.as_matrix()
    global_vector =  np.dot(rotation_matrix,body_vector)
    return global_vector


def remove_gravity_acceleration(imu_data):
    gravity_local = np.array([0, 0, 9.81])  # in m/s^2, adjust as needed
    for i in range(0,len(imu_data['timestamp'])):

        # gravity_local = np.dot(rotation_matrix, gravity_local) # 世界坐标系下的重力加速度转换为即梯坐标系
        imu_data['linear_acceleration'][i] = convert_to_global(imu_data['linear_acceleration'][i],imu_data['Quaternion'][i])

        # Remove gravity component from acceleration
        imu_data['linear_acceleration'][i] -= gravity_local


    return imu_data

def merge_dist_matrix(mat_i:np.ndarray, mat_j:np.ndarray):
    if mat_i.shape != mat_j.shape:
        return mat_i

    rows = mat_i.shape[0]
    cols = mat_i.shape[1]
    mat_ret = np.zeros([rows, cols])

    for i in range(0, rows):
        for j in range(0, cols):
            if mat_i[i][j] != 0 and mat_j[i][j] != 0:
                mat_ret[i][j] = (mat_i[i][j] + mat_j[i][j]) / 2
            elif mat_i[i][j] == 0 and mat_j[i][j] != 0:
                mat_ret[i][j] = mat_j[i][j]
            elif mat_i[i][j] != 0 and mat_j[i][j] == 0:
                mat_ret[i][j] = mat_i[i][j]

    return mat_ret