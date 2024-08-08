import numpy as np
import math


x,y = 1,1
yaw = -math.pi/2 * 1.5
p_star = np.array([0, 3])
p = np.array([x, y])
cy = math.cos(yaw)
sy = math.sin(yaw)
vx = np.linalg.norm(p_star - p) * np.matrix.dot(p_star - p, np.array([cy, sy]))

print(np.linalg.norm(p_star - p))
print( np.matrix.dot(p_star - p, np.array([cy, sy])))