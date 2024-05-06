"""https://zhuanlan.zhihu.com/p/128155013"""
import numpy as np
from spatialmath import *
from roboticstoolbox import *
from spatialmath.base import *

pi = np.pi
deg2rad = pi / 180


class get_compensation_value(object):

    def __init__(self, theta: list):
        """
        十节机械臂,传入一组关节配置，计算电机补偿值

            :param theta: 一组十电机的关节配置
            :type theta: list or numpy.ndarray
            :return Pos_value: 电机位置补偿值
            :rtype Pos_value: list
        """
        self.pi = np.pi
        self.ZERO = 0
        self.g = 9.80665
        self.LINK_MASS1 = 0.3
        self.LINK_MASS2 = 0.25
        # ------------------------------------------------------------------------------------------------------------ #
        self.lenof_link = 10
        self.length_arm = 0.081
        self.theta_start = [self.ZERO] * self.lenof_link  # 初始构型
        self.theta_goal = theta

    # noinspection PyShadowingNames
    @property
    def getPos_value(self):
        DHs = []
        for i in range(self.lenof_link - 1):
            DHs.append(RevoluteDH(a=self.length_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
        DHs.append(RevoluteDH(a=0.081, alpha=0, qlim=[-80 * pi / 180, 80 * pi / 180]))
        snake = DHRobot(DHs, name='arm')
        # theta_goal = snake.toradians(self.theta_goal)
        # snake.teach(self.theta_goal, limits=[0, 1.0, -0.5, 0.5, -0.5, 0.5])

        sol = jtraj(snake.q, self.theta_goal, 100)
        T_all = snake.fkine_all(self.theta_goal)

        # ######################################——————————求关节力臂——————————########################################## #
        '''求解每个关节所受力矩的质心坐标'''
        '''常量部分'''
        T_mass = transl(0.0405, 0, 0)  # 质心矩阵
        T_all_zeroPos = snake.fkine_all(self.theta_start)  # 零位矩阵

        ####################################################
        matrixs_all = []
        # 最后一节没有电机，只有空连杆，所以009号电机重力扰动近似为零，只求001到008号电机的重力矩
        for i in range(1, self.lenof_link - 1):
            T_temp = T_all_zeroPos[i].A
            T_temp_inv = np.linalg.inv(T_temp)
            centroid = []
            '''坐标变换：将W坐标系转换为电机所在坐标系，并记录转换后的每个关节质心的三维坐标'''
            for T in T_all_zeroPos:
                Ta = T.A
                # T_ex = np.dot(T1_inv, Ta)  # W坐标系变换
                T_ex = np.dot(np.dot(T_temp_inv, Ta), T_mass)  # 质心坐标相对于当前W坐标系的齐次变换矩阵
                centroid.append(T_ex[:3, 3])
            '''存储有效的纯位移矩阵'''
            matrixs = []
            for cent in centroid:
                mass_center_matrix = transl(cent)  # 将质心坐标转换为对应的其次变换矩阵
                matrixs.append(mass_center_matrix)
            matrixs = SE3(matrixs[i:-2])  # 带有有效位移信息的矩阵（不包括最后两个矩阵的信息，具体原因参考上文注释）
            matrixs_all.append(matrixs)
        # print(matrixs_all)
        # ###################################——————————虚拟蛇，用于求解姿态——————————######################################### #
        # 创建虚拟蛇，获取姿态信息
        theta_goal_ = [self.theta_goal[0], self.theta_goal[1], self.ZERO, self.theta_goal[2], self.ZERO,
                       self.theta_goal[3], self.ZERO, self.theta_goal[4], self.ZERO, self.theta_goal[5], self.ZERO,
                       self.theta_goal[6], self.ZERO, self.theta_goal[7], self.ZERO, self.theta_goal[8], self.ZERO,
                       self.theta_goal[9]]
        DHs_ = []
        for i in range(self.lenof_link - 1):
            DHs_.append(RevoluteDH(a=self.length_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
            DHs_.append(RevoluteDH(a=self.length_arm, alpha=0, qlim=[-80 * pi / 180, 80 * pi / 180]))
        snake_ = DHRobot(DHs_, name='arm')
        # theta_goal_ = snake_.toradians(theta_goal_)
        # snake_.teach(theta_goal_, limits=[0, 1.5, -0.5, 0.5, -0.5, 0.5])
        T_all_ = snake_.fkine_all(theta_goal_)

        # ————————————————————————————重力矢量矩阵—————————————————————————————— #
        gra_matrix1 = transl(0, 0, -self.g * (4 * self.LINK_MASS1 + 4 * self.LINK_MASS2))  # 001号电机重力矢量矩阵
        gra_matrix2 = transl(0, 0, -self.g * (3 * self.LINK_MASS1 + 4 * self.LINK_MASS2))
        gra_matrix3 = transl(0, 0, -self.g * (2 * self.LINK_MASS1 + 4 * self.LINK_MASS2))
        gra_matrix4 = transl(0, 0, -self.g * (1 * self.LINK_MASS1 + 4 * self.LINK_MASS2))
        gra_matrix5 = transl(0, 0, -self.g * 4 * self.LINK_MASS2)
        gra_matrix6 = transl(0, 0, -self.g * 3 * self.LINK_MASS2)
        gra_matrix7 = transl(0, 0, -self.g * 2 * self.LINK_MASS2)
        gra_matrix8 = transl(0, 0, -self.g * 1 * self.LINK_MASS2)
        gra_matrixs = SE3([gra_matrix1, gra_matrix2, gra_matrix3, gra_matrix4, gra_matrix5, gra_matrix6, gra_matrix7,
                           gra_matrix8])

        # ####################—————————————————————————001号电机——————————————————————————##################### #
        '''求001电机所受力矩的平均质心'''
        T1 = T_all_[1].A
        T1_inv = np.linalg.inv(T1)
        T_exs = []
        '''将W坐标变换至Virtual-Snake——001号电机参考系'''
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T1_inv, Ta)
            T_ex = np.dot(np.dot(T1_inv, Ta), T_mass)
            T_exs.append(T_ex)
        # print(SE3(T_exs))
        mass_center_poses = []  # 存储各个关节质心的姿态信息
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]  # 位置信息置零
            mass_center_poses.append(mass_center_pose)
        # print(SE3(mass_center_poses))
        '''求001号电机所受力的平均质心的xyz坐标'''
        '''求每个关节的质心坐标'''
        C = []
        k = 2
        for i in range(len(matrixs_all[0])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[0][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        '''分别求机械臂两部分的平均质心坐标'''
        XH540_mass_center = [((C[0][i] + C[1][i] + C[2][i] + C[3][i]) / 4) for i in range(3)]
        XH430_mass_center = [((C[4][i] + C[5][i] + C[6][i] + C[7][i]) / 4) for i in range(3)]
        # print(C)
        # print(XH540_mass_center, XH430_mass_center)
        # 质量不同，平均质心在两部分质心连线的向量上进行适量偏移
        vector_mass = [(XH430_mass_center[i] - XH540_mass_center[i]) for i in range(3)]
        W1_ALL = self.g * self.LINK_MASS1 * 4
        W_ALL = self.g * (self.LINK_MASS1 * 4 + self.LINK_MASS2 * 4)
        ratio = 1 - (W1_ALL / W_ALL)
        arm = [(value * ratio) for value in vector_mass]

        '''turque_arm1即为求解001号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm1 = [(XH540_mass_center[i] + arm[i]) for i in range(3)]
        # print(turque_arm1)

        # ####################—————————————————————————002号电机——————————————————————————##################### #
        '''求002电机所受力矩的平均质心'''
        T3 = T_all_[3].A
        T3_inv = np.linalg.inv(T3)
        T_exs = []
        '''变换W坐标系为002电机(虚拟蛇的003电机)坐标系'''
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T3_inv, Ta)
            T_ex = np.dot(np.dot(T3_inv, Ta), T_mass)
            T_exs.append(T_ex)
        # print(T_exs)
        '''获取姿态变换矩阵'''
        mass_center_poses = []
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        # print(SE3(mass_center_poses))  # 姿态矩阵
        C = []
        k = 4
        for i in range(len(matrixs_all[1])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[1][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        # exit()
        '''分别求机械臂两部分的平均质心坐标'''
        XH540_mass_center = [((C[0][i] + C[1][i] + C[2][i]) / 3) for i in range(3)]
        XH430_mass_center = [((C[3][i] + C[4][i] + C[5][i] + C[6][i]) / 4) for i in range(3)]
        # 质量不同，平均质心在两部分质心连线的向量上进行适量偏移
        '''求质心连线对应的的向量'''
        vector_mass = [(XH430_mass_center[i] - XH540_mass_center[i]) for i in range(3)]
        W1_ALL = self.g * self.LINK_MASS1 * 3
        W_ALL = self.g * (self.LINK_MASS1 * 3 + self.LINK_MASS2 * 4)
        ratio = 1 - (W1_ALL / W_ALL)
        arm = [(value * ratio) for value in vector_mass]
        '''turque_arm2即为求解002号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm2 = [(XH540_mass_center[i] + arm[i]) for i in range(3)]
        # print(turque_arm2)

        # ####################—————————————————————————003号电机——————————————————————————##################### #
        '''求003电机所受力矩的平均质心'''
        T5 = T_all_[5].A
        T5_inv = np.linalg.inv(T5)
        T_exs = []
        '''变换W坐标系为003电机坐标系'''
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T5_inv, Ta)
            T_ex = np.dot(np.dot(T5_inv, Ta), T_mass)
            T_exs.append(T_ex)
        '''获取姿态变换矩阵'''
        mass_center_poses = []
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        # print(SE3(mass_center_poses))  # 姿态矩阵
        C = []
        k = 6
        for i in range(len(matrixs_all[2])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[2][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        # exit()
        '''分别求机械臂两部分的平均质心坐标'''
        XH540_mass_center = [((C[0][i] + C[1][i]) / 2) for i in range(3)]
        XH430_mass_center = [((C[2][i] + C[3][i] + C[4][i] + C[5][i]) / 4) for i in range(3)]
        # 质量不同，平均质心在两部分质心连线的向量上进行适量偏移
        '''求质心连线对应的的向量'''
        vector_mass = [(XH430_mass_center[i] - XH540_mass_center[i]) for i in range(3)]
        W1_ALL = self.g * self.LINK_MASS1 * 2
        W_ALL = self.g * (self.LINK_MASS1 * 2 + self.LINK_MASS2 * 4)
        ratio = 1 - (W1_ALL / W_ALL)
        arm = [(value * ratio) for value in vector_mass]
        '''turque_arm3即为求解003号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm3 = [(XH540_mass_center[i] + arm[i]) for i in range(3)]
        # print(turque_arm3)

        # ####################—————————————————————————004号电机——————————————————————————##################### #
        '''求004电机所受力矩的平均质心'''
        T7 = T_all_[7].A
        T7_inv = np.linalg.inv(T7)
        T_exs = []
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T7_inv, Ta)
            T_ex = np.dot(np.dot(T7_inv, Ta), T_mass)
            T_exs.append(T_ex)
        '''获取姿态变换矩阵'''
        mass_center_poses = []
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        # print(SE3(mass_center_poses))  # 姿态矩阵
        C = []
        k = 8
        for i in range(len(matrixs_all[3])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[3][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        XH540_mass_center = [(C[0][i] / 2) for i in range(3)]
        XH430_mass_center = [((C[1][i] + C[2][i] + C[3][i] + C[4][i]) / 4) for i in range(3)]
        # 质量不同，平均质心在两部分质心连线的向量上进行适量偏移
        '''求质心连线对应的的向量'''
        vector_mass = [(XH430_mass_center[i] - XH540_mass_center[i]) for i in range(3)]
        W1_ALL = self.g * self.LINK_MASS1 * 1
        W_ALL = self.g * (self.LINK_MASS1 * 1 + self.LINK_MASS2 * 4)
        ratio = 1 - (W1_ALL / W_ALL)
        arm = [(value * ratio) for value in vector_mass]
        '''turque_arm4即为求解004号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm4 = [(XH540_mass_center[i] + arm[i]) for i in range(3)]
        # print(turque_arm4)

        # ####################—————————————————————————005号电机——————————————————————————##################### #
        '''求005电机所受力矩的平均质心'''
        T9 = T_all_[9].A
        T9_inv = np.linalg.inv(T9)
        T_exs = []
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T9_inv, Ta)
            T_ex = np.dot(np.dot(T9_inv, Ta), T_mass)
            T_exs.append(T_ex)
        mass_center_poses = []  # 姿态矩阵
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        C = []
        k = 10
        for i in range(len(matrixs_all[4])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[4][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        # XH540转接XH430，XH540后续不再出现
        '''此时，不再考虑重心偏移问题'''
        XH430_mass_center = [((C[0][i] + C[1][i] + C[2][i] + C[3][i]) / len(C)) for i in range(3)]
        '''turque_arm5即为求解005号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm5 = XH430_mass_center.copy()
        # print(turque_arm5)

        # ####################—————————————————————————006号电机——————————————————————————##################### #
        '''求006电机所受力矩的平均质心'''
        T11 = T_all_[11].A
        T11_inv = np.linalg.inv(T11)
        T_exs = []
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T9_inv, Ta)
            T_ex = np.dot(np.dot(T11_inv, Ta), T_mass)
            T_exs.append(T_ex)
        mass_center_poses = []  # 姿态矩阵
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        C = []
        k = 12
        for i in range(len(matrixs_all[5])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[5][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        '''此时，不再考虑重心偏移问题'''
        XH430_mass_center = [((C[0][i] + C[1][i] + C[2][i]) / len(C)) for i in range(3)]
        '''turque_arm6即为求解006号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm6 = XH430_mass_center.copy()
        # print(turque_arm6)

        # ####################—————————————————————————007号电机——————————————————————————##################### #
        '''求007电机所受力矩的平均质心'''
        T13 = T_all_[13].A
        T13_inv = np.linalg.inv(T13)
        T_exs = []
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T9_inv, Ta)
            T_ex = np.dot(np.dot(T13_inv, Ta), T_mass)
            T_exs.append(T_ex)
        mass_center_poses = []  # 姿态矩阵
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        C = []
        k = 14
        for i in range(len(matrixs_all[6])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[6][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        '''此时，不再考虑重心偏移问题'''
        XH430_mass_center = [((C[0][i] + C[1][i]) / len(C)) for i in range(3)]
        '''turque_arm7即为求解007号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm7 = XH430_mass_center.copy()
        # print(turque_arm7)

        # ####################—————————————————————————008号电机——————————————————————————##################### #
        '''求008电机所受力矩的平均质心'''
        T15 = T_all_[15].A
        T15_inv = np.linalg.inv(T15)
        T_exs = []
        for T in T_all_:
            Ta = T.A
            # T_ex = np.dot(T9_inv, Ta)
            T_ex = np.dot(np.dot(T15_inv, Ta), T_mass)
            T_exs.append(T_ex)
        mass_center_poses = []  # 姿态矩阵
        for T in T_exs:
            mass_center_pose = T[:, :3]
            mass_center_pose = np.c_[mass_center_pose, [0, 0, 0, 1]]
            mass_center_poses.append(mass_center_pose)
        C = []
        k = 16
        for i in range(len(matrixs_all[7])):
            j = k
            mass_center_xyz = np.dot(mass_center_poses[i + j], matrixs_all[7][i])
            k += 1
            C.append(mass_center_xyz[:3, 3])
        # print(C)
        '''此时，不再考虑重心偏移问题'''
        XH430_mass_center = [(C[0][i] / len(C)) for i in range(3)]
        '''turque_arm8即为求解008号电机关节力矩的平均质心三维坐标，其X轴分量就是有效力臂长度'''
        turque_arm8 = XH430_mass_center.copy()
        # print(turque_arm8)

        # #######################################——————————求有效重力荷载——————————########################################## #
        # ####################—————————————————————————001号电机——————————————————————————##################### #
        Tx = T_all_[2].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        T_Wrench1 = SE3(np.dot(Tx_copy, gra_matrixs[0]))
        # print(T_Wrench1)

        # ####################—————————————————————————002号电机——————————————————————————##################### #
        Tx = T_all_[4].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench2 = SE3(np.dot(Tx_copy_inv, gra_matrixs[1]))
        # print(T_Wrench2)

        # ####################—————————————————————————003号电机——————————————————————————##################### #
        Tx = T_all_[6].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench3 = SE3(np.dot(Tx_copy_inv, gra_matrixs[2]))
        # T_Wrench3 = SE3(np.dot(Tx_copy_inv, gra_matrix3))
        # print(T_Wrench3)

        # ####################—————————————————————————004号电机——————————————————————————##################### #
        Tx = T_all_[8].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench4 = SE3(np.dot(Tx_copy_inv, gra_matrixs[3]))
        # print(T_Wrench4)

        # ####################—————————————————————————005号电机——————————————————————————##################### #
        Tx = T_all_[10].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench5 = SE3(np.dot(Tx_copy_inv, gra_matrixs[4]))
        # print(T_Wrench5)

        # ####################—————————————————————————006号电机——————————————————————————##################### #
        Tx = T_all_[12].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench6 = SE3(np.dot(Tx_copy_inv, gra_matrixs[5]))
        # print(T_Wrench6)

        # ####################—————————————————————————007号电机——————————————————————————##################### #
        Tx = T_all_[14].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench7 = SE3(np.dot(Tx_copy_inv, gra_matrixs[6]))
        # print(T_Wrench7)

        # ####################—————————————————————————008号电机——————————————————————————##################### #
        Tx = T_all_[16].A
        Tx_copy = Tx.copy()
        Tx_copy[:3, 3] = [0, 0, 0]
        Tx_copy_inv = np.linalg.inv(Tx_copy)
        T_Wrench8 = SE3(np.dot(Tx_copy_inv, gra_matrixs[7]))
        # print(T_Wrench8)

        # #######################################————————————求解关节力矩————————————####################################### #
        '''取出有效重力荷载'''
        F_g1, F_g2, F_g3, F_g4, F_g5, F_g6, F_g7, F_g8 = T_Wrench1.A[1, 3], T_Wrench2.A[1, 3], T_Wrench3.A[1, 3], \
                                                         T_Wrench4.A[1, 3], T_Wrench5.A[1, 3], T_Wrench6.A[1, 3], \
                                                         T_Wrench7.A[1, 3], T_Wrench8.A[1, 3]
        F_g = [F_g1, F_g2, F_g3, F_g4, F_g5, F_g6, F_g7, F_g8]
        '''获取有效重力臂'''
        F_arm1, F_arm2, F_arm3, F_arm4, F_arm5, F_arm6, F_arm7, F_arm8 = turque_arm1[0], turque_arm2[0], \
                                                                         turque_arm3[0], turque_arm4[0], \
                                                                         turque_arm5[0], turque_arm6[0], \
                                                                         turque_arm7[0], turque_arm8[0]
        F_arm = [F_arm1, F_arm2, F_arm3, F_arm4, F_arm5, F_arm6, F_arm7, F_arm8]
        '''计算每一个关节的重力矩'''
        Turque = [(F_g[i] * F_arm[i]) for i in range(len(F_g))]
        # print(Turque)
        Pos_value = [int(t * 25 + 0.5) for t in Turque]  # +0.5:四舍五入
        for i in range(1, len(Pos_value)):
            Pos_value[i] = -Pos_value[i]
        Pos_value = [0] + Pos_value  # 第一个电机补零
        Pos_value.append(0)  # 最后一个电机补零
        return Pos_value


if __name__ == '__main__':
    # theta_end = [4.4498, -23.7551, 5.6507, 62.1845, -9.4642, -25.9000, -8.6671, -11.6551, -83.6463, 10.1216]
    # theta_end = np.deg2rad(theta_end)
    theta_end = [0.76699039, -0.76085447, 0.73631078, 1.05998072, -1.31615552, 1.10139821, -0.96487392, 0.47400006,
                 0.98941761, -0.64273795]
    # print(theta_end)
    a = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])# [0.70835934, -0.04486729, -0.34774297, 0.47787306, -0.82538011, 0.17146629, -0.03056944, -0.22819482,
                  #  0.47938609, -0.19114846]])
    for i in range(len(a)):
        g = get_compensation_value(a[i]).getPos_value
        print(g)
