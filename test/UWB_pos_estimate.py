#输入基站的位置，他到目标的距离，以及目标位置的粗略估计，计算目标的位置
import numpy as np
from scipy.optimize import minimize

def pos_calculate(station_pos_list, dis_list, height, rough_estimate=None, max_deviation=None):
    def distance_constraint(P, station, distance):
        return np.linalg.norm(P - station) - distance

    def height_constraint(P):
        return P[2] - height

    def positive_constraint(P):
        return P[2]
    
    def deviation_constraint(P, rough_estimate, max_deviation):
        return max_deviation - np.linalg.norm(P - rough_estimate)

    # 初始猜测位置为基站的几何中心
    initial_guess = np.mean(station_pos_list, axis=0) if rough_estimate is None else rough_estimate

    constraints = [{'type': 'eq', 'fun': distance_constraint, 'args': (station_pos_list[i], dis_list[i])}
                   for i in range(len(station_pos_list))]

    constraints.append({'type': 'ineq', 'fun': positive_constraint})

    if len(station_pos_list) == 2:
        constraints.append({'type': 'eq', 'fun': height_constraint})

    # 如果提供了粗略估计和最大偏差，添加偏差约束
    if rough_estimate is not None and max_deviation is not None:
        constraints.append({'type': 'ineq', 'fun': deviation_constraint, 'args': (rough_estimate, max_deviation)})

    # 尝试使用不同的初始猜测点和放宽的容差
    tol = 1e-10  # 容差设置
    best_result = None

    for i in range(20):
        perturbation = np.random.randn(3) * 0.1
        result = minimize(
            lambda P: 0,
            initial_guess + perturbation,  # 随机扰动初始猜测点
            method='SLSQP',
            constraints=constraints,
            options={'ftol': tol, 'maxiter': 10000}
        )
        
        if result.success:
            estimated_position = result.x
            best_result = estimated_position
            if estimated_position[0] < 0:
                best_result[0] = -estimated_position[0]
            break
        else:
            tol *= 2  # 放宽容差

    if best_result is not None:
        print("目标点估算位置:", best_result)
    else:
        print("优化失败: 无法找到满足所有约束条件的解")
    
if __name__ == "__main__":
  while True:
    station_num = int(input("请输入基站的数量："))
    if station_num < 2:
      print("基站数量至少为2,请重新输入。")
    #随机生成目标的位置
    target_pos = np.random.rand(3) * 100
    if(target_pos[2]<0):
      target_pos[2]=-target_pos[2]
    if(target_pos[0]<0):
      target_pos[0]=-target_pos[0]
    
    rough_estimate = target_pos+np.random.randn(3)*5
      
    height = target_pos[2]
    station_pos_list = []
    dis_list = []
    for i in range(station_num):
      #随机生成基站的位置
      station_pos = np.random.rand(3) * 100
      station_pos_list.append(station_pos)
      #计算目标到基站的距离
      dis = np.linalg.norm(target_pos - station_pos)
      dis_list.append(dis)
    print("目标位置:", target_pos)
    print("目标的粗略估计位置:", rough_estimate)
    pos_calculate(station_pos_list, dis_list,height,rough_estimate)
    