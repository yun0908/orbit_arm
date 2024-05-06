# -*- coding: utf-8 -*-
# 哪有什么可以直接登顶的人生，只有根据反馈不断迭代的过程!

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(threshold=np.inf)


# 简单方法画出漂亮的圆柱体（半径和高度均为1）

# def random_obs_data(num: int, seed_=100, limit_xyz=(0.6, 1.4, -1, 1, 0.5, 1.5), lim_radius=(0.05, 0.15)):
#     np.random.seed(seed_)
#     lim_x, lim_y, lim_z = limit_xyz[:2], limit_xyz[2:4], limit_xyz[4:]
#     coord_arr = np.random.random_sample((4, num))
#     coord_arr = np.around(coord_arr, decimals=3)
#     radius_lis = (lim_radius[-1] - lim_radius[0]) * coord_arr[0] + lim_radius[0]
#     x_lis = (lim_x[-1] - lim_x[0]) * coord_arr[1] + lim_x[0]
#     y_lis = (lim_y[-1] - lim_y[0]) * coord_arr[2] + lim_y[0]
#     z_lis = (lim_z[-1] - lim_z[0]) * coord_arr[3] + lim_z[0]
#     coord_obs = np.array(list(zip(x_lis, y_lis, z_lis)))
#     return coord_obs, radius_lis

def noise(sigma=0.5, seed_=100, shape=(3000, 3), lim=(-0.05, 0.05)):
    np.random.seed(seed_)
    n = np.random.random_sample(shape)
    n = np.around(n, decimals=3)
    n_data = (lim[-1] - lim[0]) * n + lim[0]
    # print(n_data)
    return n_data


# noise()
# exit()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 生成圆柱数据，底面半径为r，高度为h。
# 先根据极坐标方式生成数据
u = np.linspace(0, 2 * np.pi, 50)  # 把圆分按角度为50等分
h = np.linspace(0, 0.6, 60)  # 把高度0.6均分为60份
x = np.outer(np.sin(u), np.ones(len(h))) * 0.15  # x值重复20次
y = np.outer(np.cos(u), np.ones(len(h))) * 0.15  # y值重复20次
z = np.outer(np.ones(len(u)), h)  # x，y 对应的高度
# x 50行60列，每列代表一柱的x值，每行代表横坐标
# y 50行60列,每列代表一柱的y值，每行代表纵坐标
# z 50行60列，每行表示z轴坐标，重复50遍

# print(x, sep='\n')
# print(y.shape, sep='\n')
# print(y, sep='\n')
xy = np.c_[x[:, 0], y[:, 0]]
# print(z)
xyz_point_data = []
xyz0 = np.c_[xy, z[:, 0]]
# print(xyz0)
xyz_copy = xyz0
for i in range(1, z.shape[1]):
    xyz = np.c_[xy, z[:, i]]
    xyz_copy = np.r_[xyz_copy, xyz]
    # print(xyz_copy)
    # xyz_point_data.append(xyz)
    # exit()
# xyz_point_data = np.array(xyz_point_data)
# print(xyz_copy)
noise_d = noise()
xyz_copy_temp = xyz_copy.copy()
xyz_copy = xyz_copy + noise_d
# print(xyz_copy)
xyz_copy_copy = []
for i in range(xyz_copy_temp.shape[0]):
    if xyz_copy_temp[i][-1] >= 0.3 and xyz_copy_temp[i][0] > 0 and xyz_copy_temp[i][-1] - (
            0.6 - 2 * xyz_copy_temp[i][0]) > 0:
        # if xyz_copy_temp[i][0] > 0:
        #     if xyz_copy_temp[i][-1] - (0.6 - 2 * xyz_copy_temp[i][0]) > 0:
        pass
    else:
        xyz_copy_copy.append(xyz_copy_temp[i])
xyz_copy_copy = np.array(xyz_copy_copy)
print(xyz_copy_copy.shape)
# exit()
n = noise(shape=xyz_copy_copy.shape)
xyz_copy_copy = xyz_copy_copy + n
np.savetxt("point_bemade2_noise.txt", xyz_copy_copy)

# print(xy.shape)
# Plot the surface
ax.plot_surface(x, y, z, cmap=plt.get_cmap('rainbow'))
# plt.show()
