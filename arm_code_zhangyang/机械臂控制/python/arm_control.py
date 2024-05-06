import numpy as np
from roboticstoolbox import *
from Robotics_Arm.dynamixel_control_X import *
import Robotics_Arm.Gravity_compensation_error_10_V3 as error
from spatialmath.base import *
from spatialmath import *

np.set_printoptions(threshold=np.inf)


def pos2rad(arr):
    arr_ = arr
    if isinstance(arr_, np.ndarray):
        return ((arr_ - 2048) / 2048) * pi
    else:
        arr_ = np.array(arr_)
        return ((arr_ - 2048) / 2048) * pi


pi = np.pi

if __name__ == '__main__':
    
    ########################################### 初始化 #######################################################
    dxl_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    lenof_link = 10
    a_arm = 0.081
    RK = 6
    t = np.arange(0, 1, 0.01)
    t1 = np.arange(0, 1.01, 0.01)

    '''零位构型'''
    theta_start = [0] * lenof_link
    '''中间引导构型,过渡作用'''
    theta_mid = [2604, 1299, 2318, 2188, 1679, 2849, 1231, 2638, 1551, 2226]    
    theta_mid = pos2rad(theta_mid)
    '''进入构型: 准备伸入管道，从中间引导状态移动至预备状态，此过程不能产生碰撞'''
    theta_ready = [2548, 1552, 2528, 2739, 1190, 2766, 1419, 2357, 2693, 1629]
    theta_ready = pos2rad(theta_ready)
    '''给定轨迹的最终构型'''
    '''***********弯头************'''
    '''穿大角度弯头'''
    # theta_end = [0, -18.8, -25.5, 55.3, 23.6, -30.4, 8.3, 5.9, 40, 25.2]
    '''穿直角弯头'''

    theta_end = [25.5031, -8.5507, -18.6721, 35.7715, -25.3595, 18.9978, 5, 10, -5, 5]  # 最终构型由matlab生成，然后复制到这里执行
    
    ####################################################################################################################
    
    ########################################## 规划实际运动轨迹 ###############################################################

    '''机械臂主体'''
    DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-70 * pi / 180, 70 * pi / 180])]
    for i in range(1, lenof_link):
        DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
    snake = DHRobot(DHs, name="arm")
    theta_end = snake.toradians(theta_end)
    # snake.teach(theta_end)
    # exit()
    T_goal = snake.fkine_all(theta_end)
    T_enter = T_goal[RK].copy()
    T_enter = SE3(transl(transl(np.array(T_enter))))
    # print(T_enter)
    # exit()

    """**************************直接读取现有的数据，节省时间*********************************************************"""
    # solq = np.load('data2.npy')

    '''-----------------重新解算，如不使用现成数据，才会应用此部分------------------------'''
    '''机械臂末端位姿变换至最终构型的入口处关节位姿'''
    A = 1
    if A:
        sola = jtraj(theta_mid, theta_ready, t)
        T0 = snake.fkine(sola.q[-1])
        solb = [sola.q[-1]]
        Ts = ctraj(T0, T_enter, t)
        for T in Ts:
            sol = snake.ikine_min(T, solb[-1], qlim=True, method='SLSQP')
            solb.append(sol.q)
        solb = np.array(solb)
        solq = np.r_[sola.q, solb]
        # print(np.linalg.inv(snake.fkine(solq)[0]))
        # print(snake.fkine_all(solq[-1]))
        # exit()
        # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
        # exit()
        '''推进前两节'''
        # solj = jtraj(solq[-1][-2:], theta_end[-4:-2], t1)
        solj = jtraj(solq[-1][-2:], theta_end[-4:-2], t1)
        snake2 = DHRobot(DHs[:-2])
        T0 = snake2.fkine(solq[-1][:-2])
        Ts = ctraj(T0, T_enter, t)
        solc = [solq[-1][:-2]]
        for T in Ts:
            sol = snake2.ikine_min(T, solc[-1], qlim=True, ilimit=2000, method='SLSQP')
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
            sol = snake4.ikine_min(T, sold[-1], qlim=True, method='SLSQP')
            sold.append(sol.q)
        sold = np.array(sold)
        sold = np.c_[sold, solk.q]
        solq = np.r_[solq, sold]
        # 左右摆
        # torad = pi / 180
        # leftright = 25 * torad
        # updown = 25 * torad
        # lis_bai = [solq[-1][-2] + updown, solq[-1][-1] + leftright]
        # sol___ = jtraj(solq[-1][-2:], lis_bai, 50)
        # lis_bai_1 = np.c_[[solq[-1][:-2]] * 50, sol___.q]
        # solq = np.r_[solq, lis_bai_1]
        #
        # lis_bai = [solq[-1][-2] - 2 * updown, solq[-1][-1] - 2 * leftright]
        # sol____ = jtraj(solq[-1][-2:], lis_bai, 50)
        # lis_bai_2 = np.c_[[solq[-1][:-2]] * 50, sol____.q]
        # solq = np.r_[solq, lis_bai_2]

    print(len(solq))
    # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # exit()

    ####################################################################################################################
    motor = dxlControl_X('COM5')    # 根据你们自己的COM端口号来改
    motor.open_init_port(3000000)
    motor.enable_torque(dxl_ids)
    motor.Temperature_PreWarning(60)
    now_pos = motor.present_pos_on_time(lenof_link)
    now_pos = pos2rad(np.array(now_pos))

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # sol_g_com = jtraj(now_pos, theta_start, 200)
    # before_q = np.array([sol_g_com.q[-1]]*100)
    # # print(before_q)
    # # exit()
    # pos_com = error.get_compensation_value(sol_g_com.q[-1]).getPos_value
    # las_q = motor.radarr2Pos(np.array([sol_g_com.q[-1]]))
    # las_q = las_q + pos_com
    # [lasq] = pos2rad(las_q)
    # sol_g_comq = np.r_[sol_g_com.q, before_q, sol_g_com.q[::-1]]
    # # exit()
    # # print(sol_g_comq[-1], lasq)
    # # exit()
    # after_com_q = jtraj(sol_g_comq[-1], lasq, 200)
    # sol_g_comq = np.r_[sol_g_comq, after_com_q.q]
    # pos = motor.radarr2Pos(sol_g_comq)
    # motor.move2goal(pos)
    # # snake.plot(sol_g_comq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # # print(pos_com.getPos_value)
    # exit()
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    '''--------------------------------------------------------------------------------------------------------------'''
    '''****** 补偿前执行 ******'''
    # sol__ = jtraj(now_pos, theta_mid, 200)
    # solq = np.r_[sol__.q, solq]
    # # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # # exit()
    # solq = motor.radarr2Pos(solq)
    # # solq = motor.radarr2Pos(sol__.q)
    # motor.move2goal(solq)
    # motor.read_Data()
    # motor.Temperature_PreWarning(60)
    # exit()
    # while True:
    #     motor.Temperature_PreWarning(60)

    '''*** 补偿后 ***'''
    pos_arr = [error.get_compensation_value(q).getPos_value for q in solq]
    pos_arr = np.array(pos_arr)
    solq_Pos = motor.radarr2Pos(solq)
    for i in range(len(solq_Pos)):
        solq_Pos[i] = solq_Pos[i] + pos_arr[i]
    '''将更改后的第一组关节配置转换为弧度制，以便求解补偿后的引导态各关节位姿'''
    solq_mid_pos = pos2rad(solq_Pos[0])
    '''补偿后的全部位置值 -> 弧度制'''
    solq = pos2rad(solq_Pos)
    '''电机控制'''
    '''由任意初始配置运动至引导构型'''
    sol__ = jtraj(now_pos, solq_mid_pos, 200)
    solq = np.r_[sol__.q, solq]
    print(T_goal)
    snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # snake.teach(solq[-1])
    # exit()

    solq = motor.radarr2Pos(solq)
    input('input: ')
    motor.move2goal(solq)
    # motor.read_Data()
    motor.Temperature_PreWarning(60)
    # exit()
    '''PID'''

    goal_pos = solq[-1]
    goal_pos_real = motor.present_pos_on_time(lenof_link)
    '''
        goal_pos_real：机械臂到达目标位姿时各电机的真实位置值
        goal_pos: 机械臂到达目标位姿时各电机的理论位置值（target）
    '''
    # 每个电机位置都要校准
    for i in range(len(goal_pos_real)):
        present_pos = goal_pos_real[i]
