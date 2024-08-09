# -*- coding: UTF-8 -*-

from numpy import *
import scipy
from scipy.optimize import minimize
# from pyulog import ULog
import pandas as pd
import numpy as np
import re
from scipy.spatial.transform import Rotation as R
from quaternions import Quaternion as Quaternion
from scipy.integrate import simpson, romberg, trapezoid, cumulative_trapezoid


#
kt  = 1e-6 # 转速和升力关系系数
kr  = 8e-7 # 转速和力矩关系系数
# 拉力曲线参照 https://item.taobao.com/item.htm?spm=a21n57.1.item.4.2c191cc2HrWahR&priceTId=213e377f17214550475931805e1067&utparam=%7B%22aplus_abtest%22:%22d8071d5aba29ca7094776a086f465e22%22%7D&id=704202196861&ns=1&abbucket=5&sku_properties=1627207:23681135502
k_pacer3 = (1400 - 0) / (1 - 0.1) * 0.00981  # 0.00981 克力到牛顿 https://citizenmaths.com/zh-cn/force/grams-force-to-newtons
deadzone_pacer3 = 0.1  # 点斜表示 y - y0 = k * (x - x0), y0 = 0
#
t_m_ratio = 2.131e-8 / 2.120e-6


def quaternion2euler(quaternion, is_degree=True):
    '''
    Scipy的计算太不靠谱了
    '''
    #
    # r = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])           # x y z
    # euler = r.as_euler('zyx', degrees=is_degree)                # 注意这个顺序
    # return euler                                                # w x y z
    #
    q = Quaternion(w=quaternion[0], x=quaternion[1], y=quaternion[2], z=quaternion[3])
    euler = q.get_euler()
    if is_degree:
        euler[0] = euler[0] * 180. / np.pi
        euler[1] = euler[1] * 180. / np.pi
        euler[2] = euler[2] * 180. / np.pi
    return euler

def euler2quaternion(euler, is_degree=True):
    '''
    Scipy的计算太不靠谱了
    q = q0 + q1 * i + q2 * j + q3 * k
    q = w  + x  * i + y  * j + z  * k    
    '''
    #
    # r = R.from_euler('zyx', euler, degrees=is_degree)
    # quaternion = r.as_quat()
    # return [quaternion[1], quaternion[2], quaternion[3], quaternion[0]] # w x y z
    #
    if is_degree:
        euler[0] = euler[0] * np.pi / 180.
        euler[1] = euler[1] * np.pi / 180.
        euler[2] = euler[2] * np.pi / 180.
    quat = Quaternion.from_euler(euler, axes = ['z', 'y', 'x'])
    return quat.w, quat.x, quat.y, quat.z

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

def dshot_2_T(value):
    global k_pacer3, deadzone_pacer3

    T = k_pacer3 * ((float(value) - 48) / 2000. - deadzone_pacer3)
    if T < 0:
        T = 0               # 当前阶段是不可能产生负升力的, 下同
    return T

def dshot_2_M(value):
    global k_pacer3, deadzone_pacer3, t_m_ratio

    T = k_pacer3 * ((float(value) - 48) / 2000. - deadzone_pacer3)
    M = t_m_ratio * T
    if M < 0:
        M = 0
    return M
