"""https://etptc.tiangong.edu.cn/2022/0728/c1990a78716/page.htm"""
import numpy as np
from roboticstoolbox import *
from Robotics_Arm import *
from Robotics_Arm.dynamixel_control_X import *
import Robotics_Arm.Gravity_compensation_error_10_V3 as error

np.set_printoptions(threshold=np.inf)


# class DeltaPID(object):
#     """ Incremental PID Algorithm implementation """
#
#     def __init__(self, target, cur_val, p, i, d) -> None:
#         # self.dt = dt  # Cycle interval
#         self.k_p = p
#         self.k_i = i  # Integral coefficient
#         self.k_d = d  # Differential coefficient
#         self.target = target  # The target
#         self.cur_val = cur_val
#         self._pre_error = 0
#         self._pre_pre_error = 0
#
#     def calcalate(self):
#         error = self.target - self.cur_val
#         p_change = self.k_p * (error - self._pre_error)
#         i_change = self.k_i * error
#         d_change = self.k_d * (error - 2 * self._pre_error + self._pre_error)
#         delta_output = p_change + i_change + d_change  # This increment
#         self.cur_val += delta_output  # Calculate the current position
#         self._pre_error = error
#         self._pre_pre_error = self._pre_error
#         return self.cur_val


def pos2rad(arr):
    arr_ = arr
    if isinstance(arr_, np.ndarray):
        return ((arr_ - 2048) / 2048) * pi
    else:
        arr_ = np.array(arr_)
        return ((arr_ - 2048) / 2048) * pi


pi = np.pi

if __name__ == '__main__':
    dxl_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    lenof_link = 10
    a_arm = 0.081
    RK = 6
    t = np.arange(0, 2, 0.01)
    t1 = np.arange(0, 2.01, 0.01)

    '''零位构型'''
    theta_start = [0] * lenof_link
    '''中间引导构型,过渡作用'''
    theta_mid = [2604, 1299, 2318, 2188, 1679, 2849, 1231, 2638, 1551, 2226]
    theta_mid = pos2rad(theta_mid)
    '''预备构型: 准备伸入管道，从中间引导状态移动至预备状态，此过程不能产生碰撞'''
    theta_ready = [2548, 1552, 2528, 2739, 1190, 2766, 1419, 2357, 2693, 1629]
    theta_ready = pos2rad(theta_ready)
    '''给定轨迹的最终构型'''
    # '''***********弯头************'''
    # theta_end = [-36.1063, -9.6172, 40.5703, 21.8860, 14.0583, -12.3752, -8.1897, 1.8739, -38.8894, -4.7073]
    # theta_end = [-36.1063, -9.6172, 48.5703, 21.8860, 10.0583, -6.3752, -8.1897, 1.8739, -48.8894, -4.7073]
    # theta_end = [-7.9314, -17.8397, 17.2384, 53.6759, 2.9848, -35.0033, -23.2449, -4.6662, -17.4118, 4.1781]
    '''穿大角度弯头'''
    # theta_end = [0, -18.8, -25.5, 55.3, 23.6, -30.4, 8.3, 5.9, 40, 25.2]
    '''穿直角弯头'''
    # theta_end = [0, -15.8, -20.5, 55.3, 23.6, -25.4, 30, 5.9, 60, 20.2]
    '''测数据用'''
    # 第一组构型
    # theta_end = [-20, 25, -27, -26, 19, 15, 21, -30, 0, 20]
    # 第二组构型
    # theta_end = [0, 30, -27, -30, 25, 15, 21, -25, 0, 20]
    # 第三组构型
    # theta_end = [0, 30, -20, -30, 25, 30, 30, -25, -10, 10]
    # 第四组构型
    # theta_end = [28, 20, -28.2, 20.9, -10.9, -18.2, 17.3, -22.7, -27.3, 14.5]
    # 第五组构型
    # theta_end = [28, 15, -28.2, 6, -10.9, -17.2, 17.3, -12.7, -27.3, 14.5]
    # 第六组构型
    # theta_end = [2.0752, 17.3366, 7.2081, 66.3078, 0.0596, -45.1236, -11.7699, -56.1592, -15.7509, 11.3081]
    # 第七组构型
    # theta_end = [-14.8915, 15.9019, -19.0894, 39.9868, -18.9106, -38.8952, 40.6254, -13.9245, 37.1735,
    #              -10.8402]  # (RRT轨迹)
    # 第八组
    # theta_end = [11.2586, 20.1893, 10.3437, 44.1481, 12.6967, -24.9925, 7.0461, -27.0131, -64.6906, -10.8396]  # (RRT轨迹)

    # 实验数据 ##########################################################################################################
    # RRT穿直管道
    # theta_end = [-8.2697, 3.2900, 33.3222, 64.2581, -3.4202, -61.0455, -23.7386, -23.4874, -11.3595, 34.5455]

    # 这组不行theta_end = [-37.9910, -18.5227, 12.2273, 58.1960, 52.0617, -1.3167, -13.9899, -25.7437, -3.4132, -9.9127]

    '''论文数据'''
    # theta_end = [25.5031, -10.5507, -18.6721, 35.7715, -34.3595, 18.9978, -8.0826, -17.1588, 33.2843, -13.4935]
    # theta_end = [0.7326, -0.7398, -0.4565, 1.2600, -0.8549, -0.0714, 0.0308, -0.3863, 0.4766,
    #              -0.2341]  # 2lsat-straight-line_Present_Current
    # theta_end = [-1.1559, -24.9319, -39.6991, 74.2484, 35.2094, -17.5930, 19.0308, -34.2281, -35.4835,
    #              -13.2077]  # 1-obtuse-angle_Present_Current
    theta_end = [4.4498, -23.7551, 5.6507, 62.1845, -9.4642, -25.9000, -8.6671, -11.6551, -83.6463,
                 10.1216]  # 2-right-angle_Present_Current 2-right-angle-line_Present_Position
    '''论文数据END'''
    # 这组可以，但是效果不太好
    # theta_end = [-28.5031, -16.5507, 18.6721, 35.7715, 37.3595, 18.9978, 8.0826, -17.1588, -33.2843, -13.4935]
    # theta_end = [-28.2182, -31.7869, 15.0467, 62.5986, 26.6896, -0.6888, 29.0134, 5.0958, -29.0943, -20.4551]

    '''中'''
    # theta_end = [0.7339, -0.3620, 0.0796, 0.7743, -0.6093, 0.3486, -0.7191, -0.3476, 0.5332, -0.3708]
    # theta_end = rad2deg(theta_end)
    # theta_end = [5.5389, 38.6252, 1.2780, -9.8151, -10.4427, -48.3791, -12.8461, 26.4635, -20.4830, -2.5902]
    # theta_end = [8.2179, -21.5940, 0.2786, 77.2122, -1.4437, -44.3539, -32.8405, -2.2683, -10.6504, -4.8515]
    # theta_end = [-2.9324, -16.4163, 10.1231, 73.9120, 0.6861, -29.8278, -20.2344, -32.1227, -24.1919, 11.1403]
    # theta_end = [-21.3140, -2.8883, 45.4852, 18.2752, 5.9268, -18.6123, -31.9245, 4.7488, -37.4333, -3.2067]
    # theta_end = [-26.3140, -2.8883, 45.4852, 18.2752, 5.9268, -10, -31.9245, 4.7488, -37.4333, -3.2067]
    # theta_end = [-26.3140, -2.8883, 45.4852, 18.2752, 5.9268, -21.6123, -31.9245, 4.7488, -37.4333, -3.2067]
    # theta_end = [-26.3140, -2.8883, 45.4852, 18.2752, 5.9268, -21.6123, -31.9245, 8.7488, -37.4333, -3.2067]
    # theta_end = [-26.3140, 2.8883, 38.4852, 6.2752, 5.9268, -10.6123, -20.9245, 4.7488, -37.4333, -3.2067]
    # trach结果:[0.715,-0.087,0.025]
    '''左'''
    # theta_end = [6.8293, -13.7232, -4.1760, 46.1402, -1.3428, -29.2771, -25.6514, -12.6378, -15.7902, 12.4028]
    # theta_end = [6.8293, -13.7232, -4.1760, 46.1402, -1.3428, -23.2771, -25.6514, -5.6378, -23.7902, 12.4028]
    # theta_end = [6.8293, -13.7232, -4.1760, 46.1402, -1.3428, -23.2771, -25.6514, -20.6378, -15.7902, 12.4028]
    # theta_end = [6.8293, -13.7232, -4.1760, 36.1402, -1.3428, -20.2771, -25.6514, 10.6378, -15.7902, 12.4028]
    # theta_end = [6.8293, -13.7232, -4.1760, 46.1402, -1.3428, -23.2771, -25.6514, -20.6378, -15.7902, 20.4028]
    # teach结果:[0.724,-0.134,0.038]
    '''右'''
    # theta_end = [-27.4769, -1.1642, 46.1847, 17.4041, 5.3280, -25.0729, -27.3323, 9.7435, -30.9764, -5.4782]
    # theta_end = [-27.4769, 1.1642, 46.1847, 17.4041, 5.3280, -25.0729, -27.3323, 9.7435, -30.9764, -5.4782]
    # teach结果:[0.733,-0.061,0.03]

    '''左下'''
    # theta_end = [3.0422, -20.7893, 9.4769, 63.8460, -8.1792, -18.4057, -11.7543, -23.7404, -48.4776, 13.8748]
    # theta_end = [3.0422, -20.7893, 9.4769, 63.8460, -8.1792, -26.4057, -11.7543, -15.7404, -45.4776, 18.8748]
    '''-------------------------------------------------------------------------------'''
    '''直角弯头 规格{平直段长8cm，平直段内径：(18.5-3)cm,拐弯处半径减去3mm,入口距基座距离47cm，管道中轴线比基座高：(15.5/2-3)cm}'''
    # theta_end = [5.0095, -23.5934, 5.4478, 63.2718, -10.9269, -26.8368, -9.6109, -12.1225, -81.0892,
    #              11.3682]  # 3-right-angle-return_Present_Current
    # theta_end = [6.7309, -13.1480, 6.1010, 60.1596, -13.9942, -30.4144, -14.4461, -10.8920, -70.7087, 17.7020]
    # theta_end = [6.7309, -13.1480, 6.1010, 70.1596, -13.9942, -39.4144, -14.4461, -10.8920, -70.7087, 17.7020]
    # theta_end = [6.7613, -3.6928, 3.9797, 45.2206, -12.9336, -20.9278, -17.7230, -19.0578, -75.4041, 16.1102]

    # end ##############################################################################################################
    # theta_end = [-2.3226, -20.9480, 7.3798, 77.4159, 0.0824, -27.6410, -13.0040, -27.2079, -27.9957, 4.1149]
    # theta_end = [-1.4267, -19.2847, 7.9148, 75.7212, -0.0125, -26.6035, -14.7763, -30.0757, -27.9693, 6.5422]
    # theta_end = [-1.1800, -9.7240, 7.7083, 70.0865, -0.7312, -38.6684, -18.2280, -17.2618, -23.9737, 0.9716]
    # theta_end = [3.8939, -8.5438, 5.5861, 68.0573, -1.1525, -47.3491, -30.3865, -4.3973, -14.0773, -3.0339]
    '''20cm组'''
    '''左'''
    # theta_end = [49.4316, -4.7115, -36.3930, 27.1037, -58.8720, -1.9545, 39.4974, -17.3769, 14.2402, -5.3059]
    '''上'''
    # theta_end = [42.4996, -5.4170, -22.2185, 29.5137, -51.3043, 7.3887, 3.4754, -15.1421, 26.3069, -6.5227]
    '''右'''
    # theta_end = [50.0634, -3.1709, -41.4082, 18.8458, -48.4616, 2.8742, 13.2345, -10.8955, 15.4554, -8.3217]
    # theta_end = [-49.4316, -4.7115, 36.3930, 27.1037, 58.8720, -1.9545, -39.4974, -17.3769, -14.2402, -5.3059]
    '''24cm组'''
    '''左'''
    # theta_end = [48.0387, -10.8230, -28.1887, 34.2376, -61.8190, -0.7178, 35.6034, -17.7612, 13.4419, -7.3093]
    '''上'''
    # theta_end = [40.5860, -2.5707, -19.9242, 27.3801, -47.2908, 9.8243, -1.7515, -13.0746, 27.4668, -10.9520]
    '''右'''
    # theta_end = [39.3698, -2.4681, -24.2343, 14.0522, -44.3657, 8.6282, -16.5803, -6.2443, 34.9572, -14.1888]

    # theta_end = [-10.7340, 18.6003, 30.7182, -46.0504, -54.0926, 67.4470, 39.3754, -56.8677, -23.1300, -9.3577]
    '''弯道'''
    '''---'''
    # theta_end = [3.9498, 62.9792, -12.0154, -66.2639, 3.7486, -21.1170, -11.3148, 19.5236, -32.4080,  7.9778]
    # theta_end = [0, -13, -10, 26, 16, -13, 10, 10, 25.5, -10]
    '''Note机械臂主体'''
    DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-60 * pi / 180, 60 * pi / 180])]
    for i in range(1, lenof_link):
        DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
    snake = DHRobot(DHs, name="arm")
    theta_end = snake.toradians(theta_end)
    # snake.teach(snake.q)
    # exit()
    T_goal = snake.fkine_all(theta_end)
    T_enter = T_goal[RK].copy()

    '''机械臂末端位姿变换至最终构型的入口处关节位姿'''
    sola = jtraj(theta_mid, theta_ready, t)
    T0 = snake.fkine(sola.q[-1])
    solb = [sola.q[-1]]
    Ts = ctraj(T0, T_enter, t)
    for T in Ts:
        sol = snake.ikine_min(T, solb[-1], qlim=True, method='L-BFGS-B')
        solb.append(sol.q)
    solb = np.array(solb)
    # snake.teach(solb[-1])
    solq = np.r_[sola.q, solb]

    '''推进前两节'''
    solj = jtraj(solq[-1][-2:], theta_end[-4:-2], t1)
    snake2 = DHRobot(DHs[:-2])
    T0 = snake2.fkine(solq[-1][:-2])
    Ts = ctraj(T0, T_enter, t)
    solc = [solq[-1][:-2]]
    for T in Ts:
        sol = snake2.ikine_min(T, solc[-1], qlim=True, ilimit=2000, method='L-BFGS-B')
        solc.append(sol.q)
    solc = np.array(solc)
    solc = np.c_[solc, solj.q]
    solq = np.r_[solq, solc]
    '''第二段插入'''
    solk = jtraj(solq[-1][-4:], theta_end[-4:], t1)
    snake4 = DHRobot(DHs[:-4])
    T0 = snake4.fkine(solq[-1][:-4])
    Ts = ctraj(T0, T_enter, t)
    sold = [solq[-1][:-4]]
    for T in Ts:
        sol = snake4.ikine_min(T, sold[-1], qlim=True, method='L-BFGS-B')
        sold.append(sol.q)
    sold = np.array(sold)
    sold = np.c_[sold, solk.q]
    solq = np.r_[solq, sold]
    # np.save('right-fz.txt.npy', solq)
    # np.savetxt('right-fz.txt', solq)
    # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # snake.teach(solq[-1])
    # exit()
    # lis__ = []
    # for q in solq:
    #     # print()
    #     lis__.append(snake.fkine(q))
    # # snake.plot(solq)
    # lis__ = SE3(lis__)
    # print(lis__)
    # exit()
    ####################################################################################################################
    '''*** 补偿后 ***'''
    pos_arr = [error.get_compensation_value(q).getPos_value for q in solq]
    pos_arr = np.array(pos_arr)

    motor = dxlControl_X('COM3')
    motor.open_init_port(3000000)
    motor.enable_torque(dxl_ids)
    motor.Temperature_PreWarning(60)
    now_pos = motor.present_pos_on_time(lenof_link)
    now_pos = pos2rad(np.array(now_pos))
    '''--------------------------------------------------------------------------------------------------------------'''
    '''****** 补偿前 ******'''
    # sol__ = jtraj(now_pos, theta_mid, 200)
    # solq = np.r_[sol__.q, solq]
    # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # exit()
    # solq = motor.radarr2Pos(solq)
    # # solq = motor.radarr2Pos(sol__.q)
    # motor.move2goal(solq)
    # motor.read_Data()
    # motor.Temperature_PreWarning(60)
    # exit()
    # while True:
    #     motor.Temperature_PreWarning(60)

    solq_Pos = motor.radarr2Pos(solq)
    for i in range(len(solq_Pos)):
        solq_Pos[i] = solq_Pos[i] + pos_arr[i]

    '''##############################################################################################################'''
    '''将更改后的最后一组关节配置转换为弧度制，求解补偿后的插入位姿态及各关节位姿'''
    theta_last = pos2rad(solq_Pos[-1])
    T_alllast = snake.fkine_all(theta_last)
    T_enterlast = T_alllast[RK].copy()
    T_startlast = snake.fkine(sola.q[-1])
    T_js = ctraj(T_startlast, T_enterlast, 200)
    sole = [sola.q[-1]]
    for T in T_js:
        sol = snake.ikine_min(T, sole[-1], qlim=True, ilimit=2000, method='L-BFGS-B')
        sole.append(sol.q)
    sole = np.array(sole)
    solq_last = np.r_[sola.q, sole]
    # snake.plot(solq_last, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # snake.teach(sole[-1])
    '''推进前两节'''
    solm = jtraj(solq_last[-1][-2:], theta_last[-4:-2], 201)
    snake5 = DHRobot(DHs[:-2])
    T0_last = snake5.fkine(solq_last[-1][:-2])
    T_js = ctraj(T0_last, T_enterlast, 200)
    solf = [solq_last[-1][:-2]]
    for T in T_js:
        sol = snake5.ikine_min(T, solf[-1], qlim=True, ilimit=2000, method='L-BFGS-B')
        solf.append(sol.q)
    solf = np.array(solf)
    solf_ = np.c_[solf, solm.q]
    solq_last = np.r_[solq_last, solf_]
    # snake.plot(solq_last).hold()
    # snake.teach(solq_last[-1])
    '''推进6-8节'''
    solm = jtraj(solq_last[-1][-4:], theta_last[-4:], 201)
    snake6 = DHRobot(DHs[:-4])
    T00_last = snake6.fkine(solq_last[-1][:-4])
    T_js = ctraj(T00_last, T_enterlast, 200)
    solg = [solq_last[-1][:-4]]
    for T in T_js:
        sol = snake6.ikine_min(T, solg[-1], qlim=True, ilimit=2000, method='L-BFGS-B')
        solg.append(sol.q)
    solg = np.array(solg)
    solg_ = np.c_[solg, solm.q]
    solq_last = np.r_[solq_last, solg_]

    '''当前任意位置移动到引导态'''
    solh = jtraj(now_pos, theta_mid, 200)
    solq_last = np.r_[solh.q, solq_last]
    solq_lastPos = motor.radarr2Pos(solq_last)

    '''扫描'''
    # left_angle = 12  # deg
    # up_angle = 30  # deg
    '''左上角扫描'''
    # left_up_swing = solq_last[-1].copy()
    # left_up_swing = rad2deg(left_up_swing)
    # # snake.teach(solq_last[-1])
    # # exit()
    # left_up_swing[-2] = left_up_swing[-2] + left_angle
    # left_up_swing[-1] = left_up_swing[-1] + up_angle
    # left_up_swing2rad = deg2rad(left_up_swing)
    # left_up_sol = jtraj(solq_last[-1], left_up_swing2rad, 100)
    # '''扫描'''
    # down_init = left_up_sol.q[-1]
    # down_sol2deg = rad2deg(down_init)
    # down_sol2deg[-1] = down_sol2deg[-1] - 2 * up_angle
    # down_sol2deg2rad = deg2rad(down_sol2deg)
    # down_sol = jtraj(down_init, down_sol2deg2rad, 100)
    # '''回正'''
    # return_init = down_sol.q[-1]
    # return_init2deg = rad2deg(return_init)
    # return_init2deg[-2] = return_init2deg[-2] - left_angle
    # return_init2deg[-1] = return_init2deg[-1] + up_angle
    # return_end2rad = deg2rad(return_init2deg)
    # return_sol = jtraj(return_init, return_end2rad, 100)
    # scan_solq = np.r_[solq_last, left_up_sol.q, down_sol.q, return_sol.q]
    # # solq_lastPos = motor.radarr2Pos(scan_solq)
    # '''退回去'''
    # solq_lastPos_inv = solq_lastPos[600:][::-1]
    # solq_lastPos = np.r_[solq_lastPos, solq_lastPos_inv]
    # snake.plot(solq_last, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # exit()
    # np.save('solq_right', solq_last)
    # print(solq_last)
    # exit()
    motor.move2goal(solq_lastPos)
    motor.Temperature_PreWarning(60)
    exit()
