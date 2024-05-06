from Robotics_Arm import *
from Robotics_Arm.dynamixel_control_X import *
import Robotics_Arm.Gravity_compensation_error_10_V3 as error
from spatialmath import *
from collections import namedtuple
from spatialmath.base import *
import scipy.optimize as opt
import swift


iksol = namedtuple("IKsolution", "q, success, reason, iterations, residual")

pi = np.pi

n = 10


# 最小化目标函数
def cost(q, T, weight, costfun, stiffness):
    """目标矩阵"""
    e = pose_error(snake.fkine(q).A, T) * weight
    E = (e ** 2).sum()
    if stiffness > 0:
        E += np.sum(np.diff(q) ** 2) * stiffness
    if costfun is not None:
        E += (e ** 2).sum() + costfun(q)
    return E


# 获取位姿误差，以向量形式返回
def pose_error(T, Td):
    """
    T：当前矩阵 Td：目标矩阵
    """
    d = transl(Td) - transl(T)
    R = t2r(Td) @ t2r(T).T
    u = np.r_[R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
    if iszerovec(u):  # 对角，共线
        if np.trace(R) > 0:
            aa = np.zeros((3,))
        else:
            aa = np.pi / 2 * (np.diag(R) + 1)  # !!
    else:
        aa = math.atan2(norm(u), np.trace(R) - 1) * u / norm(u)
    return np.r_[d, aa]


def joint_limit(weight, T, q0, qlim, ilimit=1000, tol=1e-16, method='L-BFGS-B', stiffness=0, costfun=None):
    q0 = getvector(q0, n)
    solutions = []
    optdict = {"maxiter": ilimit, "disp": False}
    bounds = opt.Bounds(qlim[0, :], qlim[1, :])
    for Tk in T:
        res = opt.minimize(cost, q0,
                           args=(Tk.A, weight, costfun, stiffness),
                           bounds=bounds,
                           method=method,
                           tol=tol,
                           options=optdict
                           )
        solution = iksol(res.x, res.success, res.message, res.nit, res.fun)
        solutions.append(solution)
        q0 = res.x

    if len(T) == 1:
        return solutions[0]
    return solutions


np.set_printoptions(threshold=np.inf)

if __name__ == '__main__':
    dxl_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    lenof_link = 10
    a_arm = 0.081
    t = np.arange(0, 2, 0.01)
    te = [30, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-70 * pi / 180, 70 * pi / 180])]
    for i in range(1, lenof_link):
        DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-80 * pi / 180, 80 * pi / 180]))
    snake = DHRobot(DHs, name="arm")
    # snake.teach(snake.q)
    # exit()
    # env = swift.Swift()
    # env.launch(realtime=True)
    # env.add(snake)

    # print(snake.fkine_all(snake.q))

    # exit()
    T0 = SE3(0.65, 0, 0.15)
    sol = snake.ikine_min(T0, qlim=True, options={"disp": False})
    snake.q = sol.q
    I = np.eye(10)
    qs = [sol.q]
    v = np.array([[0], ] * 6)
    # angle = qs[-1][-3] / 200
    count = 0
    param = np.array([0.15] * lenof_link)
    wr = 1 / snake.reach
    weight = np.r_[wr, wr, wr, 1, 1, 1]
    qlim = np.array([[-70 * pi / 180, -80 * pi / 180, -80 * pi / 180, -80 * pi / 180, -80 * pi / 180, -80 * pi / 180,
                      -80 * pi / 180, -80 * pi / 180, -80 * pi / 180, -80 * pi / 180],
                     [70 * pi / 180, 80 * pi / 180, 80 * pi / 180, 80 * pi / 180, 80 * pi / 180, 80 * pi / 180,
                      80 * pi / 180, 80 * pi / 180, 80 * pi / 180, 80 * pi / 180]])
    inv = 0
    for i in range(501):
        j = snake.jacobe(qs[-1])
        j_pinv = np.linalg.pinv(j)
        dH = np.array(
            [[0], [param[1]], [param[2]], [0], [param[4]], [param[5]], [param[6]], [0], [param[8]], [0]]) * 0.1
        dq = j_pinv @ v + (I - j_pinv @ j) @ dH
        dq = dq[:, 0]
        q = qs[-1] + dq
        # # # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        sol_ = joint_limit(weight, T0, q, qlim)
        tem = sol_.q - q
        if iszerovec(sol_.q - q):
            qs.append(q)
        else:
            qs.append(sol_.q)
        if i % 250 == 0:
            inv += 1
            # param = (-1) * param
            # param = np.random.randint(-12, 12, (10,), dtype=np.int32) / 100
            param = np.random.uniform(-0.16, 0.16, (10,))
            print(param)
            exit()
    qs = np.array(qs)
    # np.save('../Data/self-motion.npy', qs)

    snake.plot(qs, block=True).hold()
    # qs = qs * (180 / pi)
    exit()

    motor = dxlControl_X('COM4')
    motor.open_init_port(3000000)
    motor.enable_torque(dxl_ids)
    motor.Temperature_PreWarning(60)
    now_pos = motor.present_pos_on_time(lenof_link)
    now_pos = pos2rad(np.array(now_pos))

    pos_arr = [error.get_compensation_value(q).getPos_value for q in qs]
    pos_arr = np.array(pos_arr)
    solq_Pos = motor.radarr2Pos(qs)
    for i in range(len(solq_Pos)):
        solq_Pos[i] = solq_Pos[i] + pos_arr[i]
    '''将更改后的第一组关节配置转换为弧度制，以便求解补偿后的引导态各关节位姿'''
    # solq_mid_pos = pos2rad(solq_Pos[0])
    '''补偿后的全部位置值 -> 弧度制'''
    solq = pos2rad(solq_Pos)
    '''电机控制'''
    '''由任意初始配置运动至引导构型'''
    sol__ = jtraj(now_pos, solq[0], 200)
    solq = np.r_[sol__.q, solq]

    # ##############################################################
    # ##############################################################
    np.save('../Data/self-motion.npy', solq)
    exit()
    # ##############################################################
    # ##############################################################
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # sol1 = jtraj(now_pos, qs[0], 200)
    # solq = np.r_[sol1.q, qs]
    # snake.plot(solq, limits=[0, 1, -0.5, 0.5, -0.5, 0.5]).hold()
    # exit()
    solq = motor.radarr2Pos(solq)
    motor.move2goal(solq)
    # motor.read_Data()
    motor.Temperature_PreWarning(60)
    exit()
