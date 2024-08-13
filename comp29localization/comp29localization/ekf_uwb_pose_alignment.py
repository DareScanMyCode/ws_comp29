# -*- coding: UTF-8 -*-
import math
from math import sqrt, cos, sin
import numpy as np
from numpy import linalg, ravel, vstack, zeros, reshape
import scipy
from scipy.optimize import minimize
from comp29localization.ekf_utils import angle_between_vectors

#
k_between_frame = 0.01

def optimize_pose_in_frames(Xt_all, distance_matrix, id_list):
    """
    :param Xt_all:              【矩阵】n架无人机位置信息[ x y z ] 维度：n×3
    :param distance_matrix:     【矩阵】n架无人机相对位置信息       维度：n×n
    :param id_list: 【 向量1×n 】  参与计算的飞机ID
    :return Xt_all:             【矩阵】更新后无人机位置信息        维度：n×3
    """
    count = len(Xt_all)

    def opt_goal1(x):
        return np.sum(x**2)

    def opt_goal2(x):
        goal2 = 0
        for i in range(0, count):
            for j in range(i + 1, count):
                #
                if not (i in id_list and j in id_list):
                    continue

                if distance_matrix[i][j] != 0 or distance_matrix[j][i] != 0:
                    # 取非零距离，或取平均值
                    if distance_matrix[i][j] == 0 and distance_matrix[j][i] != 0:
                        dist = distance_matrix[j][i]
                    elif distance_matrix[j][i] == 0 and distance_matrix[i][j] != 0:
                        dist = distance_matrix[i][j]
                    elif distance_matrix[i][j] != 0 and distance_matrix[j][i] != 0:
                        dist = (distance_matrix[i][j] + distance_matrix[j][i]) / 2
                    goal2 += (dist - sqrt((Xt_all[i][0] - Xt_all[j][0] + x[2 * i] - x[2 * j]) ** 2 + (
                                        Xt_all[i][1] - Xt_all[j][1] + x[2 * i + 1] - x[2 * j + 1]) ** 2)) ** 2
        return goal2

    k1 = 5
    k2 = 1

    def opt_goal(x):
        return k1 * opt_goal1(x) + k2 * opt_goal2(x)

    initial = ravel(reshape(Xt_all[:,:2],[2*count,1]))
    result = minimize(opt_goal, initial)
    # 更新 Xt_all 使用优化结果
    optimized_positions = np.array([result.x[0::2], result.x[1::2]]).T
    Xt_all[:, :2] += optimized_positions
    # print(f"[optimize] correction amount: {optimized_positions}")
    return Xt_all

def optimize_pose_between_frames(Xt1, Xt, Dt1, Dt, id_list):
    """
    :param Xt1:     【 矩阵 3×n 】 前一时刻n架飞机世界坐标系位置 [ x y z ]
    :param Xt:      【 矩阵 3×n 】 该时刻n架飞机世界坐标系速度 [ x y z]
    :param Dt1:     【 矩阵 n×n 】 前一时刻n架飞机的距离矩阵
    :param Dt:      【 矩阵 n×n 】 该时刻n架飞机的距离矩阵
    :param id_list: 【 向量1×n 】  参与计算的飞机ID
    :return: Xt_update:   【 向量 3×n 】 更新后获得的位置信息
    """
    # # 由于时间间隔比较大，而Vicon获得的速度并不为平均速度
    # Vt1 = (Xt - Xt1) / dt
    # print(Vt1)

    count = len(Dt)
    # dist_t1 = get_d_horizon(Xt1, Dt1)
    # dist_t = get_d_horizon(Xt, Dt)
    dist_t1 = Dt1
    dist_t = Dt
    # 飞机i：Xt  飞机j：Xn
    # f:余弦定理
    def cost_function(x):
        goal = 0
        for i in range(0, count):
            for j in range(i+1, count):
                #
                # 这里是本来是用不为0的边计算的, 等于实际飞机收多个数据的时候要置零
                #if dist_t[i,j] != 0:
                if i in id_list and j in id_list and dist_t[i,j] != 0:
                    # 公共边计算方法1
                    delta_xt_xt1 = Xt[i,:2] - Xt1[i,:2]
                    delta_xt_xn  = Xt[i,:2] - Xt[j,:2]
                    theta = angle_between_vectors(delta_xt_xn,delta_xt_xt1)
                    goal1 = dist_t[i, j] ** 2 + (delta_xt_xt1[0] + x[2*i]) ** 2 + (delta_xt_xt1[1] + x[2*i+1]) ** 2 - 2 * dist_t[i, j] * sqrt(
                        (delta_xt_xt1[0] + x[2*i]) ** 2 + (delta_xt_xt1[1] + x[2*i+1]) ** 2) * cos(theta)
                    # 公共边计算方法2
                    delta_xn_xn1 = Xt[j, :2] - Xt1[j, :2]
                    delta_xt1_xn1 = Xt1[i, :2] - Xt1[j, :2]
                    theta = angle_between_vectors(delta_xt1_xn1, delta_xn_xn1)
                    goal2 = dist_t1[i, j] ** 2 + (delta_xn_xn1[0] + x[2 * j]) ** 2 + (delta_xn_xn1[1] + x[2 * j + 1]) ** 2 - 2 * dist_t1[i, j] * sqrt(
                        (delta_xn_xn1[0] + x[2 * j]) ** 2 + (delta_xn_xn1[1] + x[2 * j + 1]) ** 2) * cos(theta)
                    goal += (goal1 - goal2) ** 2
        return goal



    # 定义目标函数 k:超参数 f1：第一个等式约束（余弦定理）的结果 f2：第二个等式约束（余弦定理）的结果

    def opt_goal(x):
        return linalg.norm(x)**2 + k_between_frame * cost_function(x)

    init_guess = zeros(2*count)

    result = minimize(opt_goal, init_guess)
    # print("Objective function value at optimal solution:", result.fun)
    # 更新 Xt_all 使用优化结果
    optimized_positions = np.array([result.x[0::2], result.x[1::2]]).T
    # print('[between_frames]optimal solution:', optimized_positions)
    Xt[:, :2] += optimized_positions
    # print('[between_frames] Xt_update:', Xt)

    return Xt

def local_pos_alginment(pos_t_i_j, pos_t_i_j_last, dist_mat, dist_mat_last, id_list, uav_index_num=10):
    '''
    Params:
        post_t_i_j
        post_t_i_j_last [[x_this, y_this, z_this],  [x_i, y_i, z_j],  [x_j, y_j, z_j]]
        dist_mat
        dist_mat_last uwb pos
    '''
    this_uav_id = id_list[0]
    uav_id_i    = id_list[1]
    uav_id_j    = id_list[2]

    center_pos = np.zeros(3)
    for i in range(0, 3):                                               # 只用三架飞机运算
        for j in range(0, 3):                                           # x y z
            center_pos[j] = center_pos[j] + pos_t_i_j[i][j] / 3   # 千万别加括号, 取平均
    #
    center_pos_last = np.zeros(3)
    for i in range(0, 3):
        for j in range(0, 3):
            center_pos_last[j] = center_pos_last[j] + pos_t_i_j_last[i][j] / 3
    #
    # CIRITCAL
    # 减掉重心后计算
    local_pos       = np.zeros([3, 3])          # x y z
    local_pos_last  = np.zeros([3, 3])
    delta_xyz_local = np.zeros([3, 3])
    for i in range(0, 3):
        for j in range(0, 3):
            local_pos[i][j]      = pos_t_i_j[i][j]      - center_pos[j]
            local_pos_last[i][j] = pos_t_i_j_last[i][j] - center_pos_last[j]
            #
            delta_xyz_local[i][j] = pos_t_i_j[i][j] - pos_t_i_j_last[i][j]
    #
    dist_mat_h      = np.zeros(dist_mat.shape)
    dist_mat_last_h = np.zeros(dist_mat.shape)
    for i in range(0, dist_mat.shape[0]):                   # i 测 j
        uav_index = 0
        for j in range(0, 3):
            if i == id_list[j]:
                uav_index = j           # local pos 中uav index

        for j in range(0, dist_mat.shape[1]):
            if i in id_list and j in id_list:
                if math.fabs(local_pos[uav_index][2]) <= 1.5 and dist_mat[i][j] ** 2 > local_pos[uav_index][2] ** 2:                      # 因为有时候高度会测不准
                    dist_mat_h[i][j]      = sqrt(dist_mat[i][j] ** 2      - local_pos[uav_index][2] ** 2)               # 计算水平距离
                else:
                    dist_mat_h[i][j] = dist_mat[uav_index][j]
                if math.fabs(local_pos_last[uav_index][2]) <= 1.5 and dist_mat_last[i][j] ** 2 > local_pos_last[uav_index][2] ** 2:
                    dist_mat_last_h[i][j] = sqrt(dist_mat_last[i][j] ** 2 - local_pos_last[uav_index][2] ** 2)
                else:
                    dist_mat_last_h[i][j] = dist_mat[uav_index][j]


    def get_avg_val_from_dist_mat(dist_mat_h, this_uav_id, uav_id_i, uav_id_j):
        dist_t_i_ref = 0.
        if dist_mat_h[this_uav_id][uav_id_i] != 0 and dist_mat_h[uav_id_i][this_uav_id] != 0:
            dist_t_i_ref = (dist_mat_h[this_uav_id][uav_id_i] + dist_mat_h[uav_id_i][this_uav_id]) / 2
        elif dist_mat_h[this_uav_id][uav_id_i] == 0:
            dist_t_i_ref = dist_mat_h[uav_id_i][this_uav_id]
        elif dist_mat_h[uav_id_i][this_uav_id] == 0:
            dist_t_i_ref = dist_mat_h[this_uav_id][uav_id_i]

        dist_t_j_ref = 0.
        if dist_mat_h[this_uav_id][uav_id_j] != 0 and dist_mat_h[uav_id_j][this_uav_id] != 0:
            dist_t_j_ref = (dist_mat_h[this_uav_id][uav_id_j] + dist_mat_h[uav_id_j][this_uav_id]) / 2
        elif dist_mat_h[this_uav_id][uav_id_j] == 0:
            dist_t_j_ref = dist_mat_h[uav_id_j][this_uav_id]
        elif dist_mat_h[uav_id_j][this_uav_id] == 0:
            dist_t_j_ref = dist_mat_h[this_uav_id][uav_id_j]

        dist_i_j_ref = 0.
        if dist_mat_h[uav_id_i][uav_id_j] != 0 and dist_mat_h[uav_id_j][uav_id_i] != 0:
            dist_i_j_ref = (dist_mat_h[uav_id_i][uav_id_j] + dist_mat_h[uav_id_j][uav_id_i]) / 2
        elif dist_mat_h[uav_id_i][uav_id_j] == 0:
            dist_i_j_ref = dist_mat_h[uav_id_j][uav_id_i]
        elif dist_mat_h[uav_id_j][uav_id_i] == 0:
            dist_i_j_ref = dist_mat_h[uav_id_i][uav_id_j]

        return dist_t_i_ref, dist_t_j_ref, dist_i_j_ref

    '''
        x: [x_this, y_this, x_i, y_i, x_j, y_j]
    '''
    def j_delta_pos(x):
        #
        # 和当前位置相差尽可能小
        #
        dx_t = x[0] - local_pos[0][0]
        dy_t = x[1] - local_pos[0][1]
        #
        dx_i = x[2] - local_pos[1][0]
        dy_i = x[3] - local_pos[1][1]
        #
        dx_j = x[0] - local_pos[2][0]
        dy_j = x[1] - local_pos[2][1]
        #
        j = dx_t ** 2 + dy_t ** 2 + dx_i ** 2 + dy_i ** 2 + dx_j ** 2 + dy_j ** 2
        return j

    def j_edge(x):
        #
        x_t = x[0]
        y_t = x[1]
        x_i = x[2]
        y_i = x[3]
        x_j = x[4]
        y_j = x[5]
        #
        # current this_uav -> uav_i
        dist_t_i_2 = (x_t - x_i) ** 2 + (y_t - y_i) ** 2
        dist_t_j_2 = (x_t - x_j) ** 2 + (y_t - y_j) ** 2
        dist_i_j_2 = (x_i - x_j) ** 2 + (y_i - y_j) ** 2

        dist_t_i_ref, dist_t_j_ref, dist_i_j_ref = get_avg_val_from_dist_mat(dist_mat_h, this_uav_id, uav_id_i, uav_id_j)

        J = 0
        if dist_t_i_ref != 0.:
            J = J + (dist_t_i_2 - dist_t_i_ref ** 2) ** 2
        if dist_t_j_ref != 0.:
            J = J + (dist_t_j_2 - dist_t_j_ref ** 2) ** 2
        if dist_t_i_ref != 0.:
            J = J + (dist_i_j_2 - dist_i_j_ref ** 2) ** 2

        return J

    def j_angle(x):
        #
        x_t = x[0]
        y_t = x[1]
        x_i = x[2]
        y_i = x[3]
        x_j = x[4]
        y_j = x[5]
        #
        # current this_uav -> uav_i
        d_ti = sqrt((x_t - x_i) ** 2 + (y_t - y_i) ** 2)
        d_tj = sqrt((x_t - x_j) ** 2 + (y_t - y_j) ** 2)
        d_ij = sqrt((x_i - x_j) ** 2 + (y_i - y_j) ** 2)

        #
        # ref
        d_ti_p, d_tj_p, d_ij_p = get_avg_val_from_dist_mat(dist_mat_h, this_uav_id, uav_id_i, uav_id_j)
        if not (d_ti_p != 0 and d_tj_p != 0 and d_ij_p != 0):
            return None

        #
        # current
        # 对边为t <-> i
        cos_t_i = d_tj ** 2 + d_ij ** 2 - d_ti ** 2 / (2 * d_tj * d_ij)
        #
        # 对边为t <-> j
        cos_t_j = d_ti ** 2 + d_ij ** 2 - d_tj ** 2 / (2 * d_ti * d_ij)
        #
        # 对边为i <-> j
        cos_i_j = d_ti ** 2 + d_tj ** 2 - d_ij ** 2 / (2 * d_ti * d_tj)

        # REF
        # 对边为t <-> i
        cos_t_i_ref = d_tj_p ** 2 + d_ij_p ** 2 - d_ti_p ** 2 / (2 * d_tj_p * d_ij_p)
        #
        # 对边为t <-> j
        cos_t_j_ref = d_ti_p ** 2 + d_ij_p ** 2 - d_tj_p ** 2 / (2 * d_ti_p * d_ij_p)
        #
        # 对边为i <-> j
        cos_i_j_ref = d_ti_p ** 2 + d_tj_p ** 2 - d_ij_p ** 2 / (2 * d_ti_p * d_tj_p)

        #
        J = 0
        if np.isreal(cos_t_i_ref):
            J = J + (cos_t_i - cos_t_i_ref ** 2) ** 2
        if np.isreal(cos_t_j_ref):
            J = J + (cos_t_j - cos_t_j_ref ** 2) ** 2
        if np.isreal(cos_i_j_ref):
            J = J + (cos_t_j - cos_t_j_ref ** 2) ** 2
        if not np.isreal(cos_t_i_ref) and not np.isreal(cos_t_j_ref) and not np.isreal(cos_i_j_ref):
            return None
        else:
            return J

    def j_angle_last(x):
        #
        x_t = x[0] - delta_xyz_local[0][0]
        y_t = x[1] - delta_xyz_local[0][1]
        x_i = x[2] - delta_xyz_local[1][0]
        y_i = x[3] - delta_xyz_local[1][1]
        x_j = x[4] - delta_xyz_local[2][0]
        y_j = x[5] - delta_xyz_local[2][1]
        #
        # current this_uav -> uav_i
        d_ti = sqrt((x_t - x_i) ** 2 + (y_t - y_i) ** 2)
        d_tj = sqrt((x_t - x_j) ** 2 + (y_t - y_j) ** 2)
        d_ij = sqrt((x_i - x_j) ** 2 + (y_i - y_j) ** 2)

        #
        # ref
        d_ti_p, d_tj_p, d_ij_p = get_avg_val_from_dist_mat(dist_mat_last_h, this_uav_id, uav_id_i, uav_id_j)
        if not (d_ti_p != 0 and d_tj_p != 0 and d_ij_p != 0):
            return None

        #
        # current
        # 对边为t <-> i
        cos_t_i = d_tj ** 2 + d_ij ** 2 - d_ti ** 2 / (2 * d_tj * d_ij)
        #
        # 对边为t <-> j
        cos_t_j = d_ti ** 2 + d_ij ** 2 - d_tj ** 2 / (2 * d_ti * d_ij)
        #
        # 对边为i <-> j
        cos_i_j = d_ti ** 2 + d_tj ** 2 - d_ij ** 2 / (2 * d_ti * d_tj)

        # REF
        # 对边为t <-> i
        cos_t_i_ref = d_tj_p ** 2 + d_ij_p ** 2 - d_ti_p ** 2 / (2 * d_tj_p * d_ij_p)
        #
        # 对边为t <-> j
        cos_t_j_ref = d_ti_p ** 2 + d_ij_p ** 2 - d_tj_p ** 2 / (2 * d_ti_p * d_ij_p)
        #
        # 对边为i <-> j
        cos_i_j_ref = d_ti_p ** 2 + d_tj_p ** 2 - d_ij_p ** 2 / (2 * d_ti_p * d_tj_p)

        #
        J = 0
        if np.isreal(cos_t_i_ref):
            J = J + (cos_t_i - cos_t_i_ref ** 2) ** 2
        if np.isreal(cos_t_j_ref):
            J = J + (cos_t_j - cos_t_j_ref ** 2) ** 2
        if np.isreal(cos_i_j_ref):
            J = J + (cos_t_j - cos_t_j_ref ** 2) ** 2
        if not np.isreal(cos_t_i_ref) and not np.isreal(cos_t_j_ref) and not np.isreal(cos_i_j_ref):
            return None
        else:
            return J

    # 取两个三角形尽可能全等, 或者理解为形变最小
    k_bias = 2.25
    k_angle = 5.5
    k_dist  = 1.25
    def cost_function(x):
        J = j_delta_pos(x) * k_bias + j_edge(x) * k_dist
        #
        j_angle_x = j_angle(x)
        if j_angle_x != None:
            J = J + j_angle_x * k_angle
        #
        j_angle_x_last = j_angle_last(x)
        if j_angle_x_last != None:
            J = J + j_angle_x_last * k_angle

        return J

    init_guess = np.array([local_pos[0][0], local_pos[0][1], local_pos[1][0], local_pos[1][1], local_pos[2][0], local_pos[2][1]])

    result = minimize(cost_function, init_guess)
    # print(result)

    #
    # current_uav_pos = np.zeros([3,3])
    # current_uav_pos[0][0] = result.x[0]
    # current_uav_pos[0][1] = result.x[1]
    # current_uav_pos[0][2] = local_pos[0][2]
    # #
    # current_uav_pos[1][0] = result.x[2]
    # current_uav_pos[1][1] = result.x[3]
    # current_uav_pos[1][2] = local_pos[1][2]
    # #
    # current_uav_pos[2][0] = result.x[4]
    # current_uav_pos[2][1] = result.x[5]
    # current_uav_pos[2][2] = local_pos[2][2]
    current_uav_pos = np.zeros([uav_index_num, 3])
    #
    current_uav_pos[id_list[0]][0] = result.x[0]
    current_uav_pos[id_list[0]][1] = result.x[1]
    current_uav_pos[id_list[0]][2] = local_pos[0][2]
    #
    current_uav_pos[id_list[1]][0] = result.x[2]
    current_uav_pos[id_list[1]][1] = result.x[3]
    current_uav_pos[id_list[1]][2] = local_pos[1][2]
    #
    current_uav_pos[id_list[2]][0] = result.x[4]
    current_uav_pos[id_list[2]][1] = result.x[5]
    current_uav_pos[id_list[2]][2] = local_pos[2][2]

    return center_pos, current_uav_pos
