from roboticstoolbox import *
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

# DH参数建模
a_arm = 0.081
pi = np.pi
num_link = 10
DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-60 * pi / 180, 60 * pi / 180])]
for i in range(1, num_link):
    DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
snake = DHRobot(DHs, name="arm")

# 正解(由构型得末端位姿)
q0 = [0, pi / 6, 0, -pi / 50, pi / 6, 0, -pi / 50, pi / 6, 0, -pi / 5]  # 机械臂构型

t0 = np.arange(0, 1, 0.005)

goal = [0] * 10
start = [2112, 1458, 1997, 2142, 2062, 2048, 2093, 1968, 1943, 1769]  # 电机位置数值
start = np.array(start)
start = (start - 2048) / 4096 * 2 * pi  # 关节角度值
jpath = jtraj(start, q0, t0)
# snake.plot(jpath.q)

# input()
pos = (jpath.q / (2 * pi)) * 4096 + 2048
pos = pos.astype(np.int32)
pos = pos.tolist()


############################## 电机控制 ################################
######################################################################

from bycontrol2 import *

ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

check()
start = read_position(ID)  # 电机位置数值
mid = [2112, 1458, 1997, 2142, 2062, 2048, 2093, 1968, 1943, 1769]  # 电机位置数值
s1 = cal_d(start, mid, 200)

# 抬起一段角度
for i in range(len(s1)):
    do(s1[i])
    time.sleep(0.01)

input('按下回车键')
# 运行仿真轨迹
for i in range(len(pos)):
    do(pos[i])
    time.sleep(0.05)

