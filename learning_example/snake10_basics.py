from roboticstoolbox import *
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

# DH参数建模
a_arm = 0.081;
pi = np.pi;
num_link = 10
DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-60 * pi / 180, 60 * pi / 180])]
for i in range(1, num_link):
    DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
snake = DHRobot(DHs, name="arm")

# 正解(由构型得末端位姿)
q0 = [0, pi / 6, 0, -pi / 50, pi / 6, 0, -pi / 50, pi / 6, 0, -pi / 5]  # 机械臂构型
# fig = snake.plot(q0)
T0 = snake.fkine(q0)
print('T0(齐次变换矩阵)为:\n', T0)

# 逆解（由末端位姿得构型）
q1 = snake.ikine_min(T0)
# snake.plot(q1.q)

# 关节空间轨迹规划
t0 = np.arange(0, 1, 0.01)
jpath = jtraj(snake.q, [1 / 8 * pi] * snake.n, t0)
# 解除下述注释查看效果
# snake.plot(jpath.q)

# 笛卡尔空间轨迹规划
T1 = SE3([0.8, 0.5, 0.4]) * SE3.OA([0.5, 0, 0], [0, 0, 0.5])
# snake.ikine_min(T1)
t1 = np.arange(0, 1, 0.02)
cpath = ctraj(T0, T1, t1)
s = np.zeros([len(cpath), snake.n])
for i in range(len(cpath)):
    s[i] = snake.ikine_min(cpath[i]).q
snake.plot(s)


