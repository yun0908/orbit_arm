# !/usr/bin/env python
import math
import numpy as np
from spatialmath import *
from roboticstoolbox import *
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialgeometry import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import sys

# Control table address
ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132
ADDR_PRO_PRESENT_CURRENT = 126  # 当前电流
ADDR_PRO_PRESENT_TEMPERATURE = 146

# Data Byte Length
LEN_PRO_CURRENT = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4
LEN_PRO_PRESENT_TEMPERATURE = 1

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel
BAUDRATE = 3000000  # Dynamixel default baudrate : 57600 (bps!!!!!)
DEVICENAME = 'COM3'  # Check which port is being used on your controller  (端口!!!!!)
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# DXL_POSITION_VALUE          = []           # Dynamixel will rotate between this value

DXL_MOVING_STATUS_THRESHOLD = 2  # Dynamixel moving status threshold
# dxl_goal_position = DXL_POSITION_VALUE     # Goal position
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
pi = math.pi


def cal_pi_to_2048(a):  # 把弧度制变成2048制
    return np.multiply(np.multiply(a, 1 / (2 * pi)), 4096) + 2048
    # return (np.multiply(np.multiply(a, 1 / (2 * pi)), 4096) + 2048.5).astype(np.int64)


# 准备啦,开力矩
def ready(n):
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        sys.exit()
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        sys.exit()
    for id in range(n):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)


# 位移到指定角度
def work(dxl_goal_position):
    # Add parameter storage for Dynamixel#1 present position value
    for id in range(len(dxl_goal_position)):
        dxl_addparam_result = groupSyncRead.addParam(id)
        if not dxl_addparam_result:
            print("[ID:%03d] groupSyncRead addparam failed" % id)
            sys.exit()
    for id in range(len(dxl_goal_position)):
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD((int)(dxl_goal_position[id]))),
                               DXL_HIBYTE(DXL_LOWORD((int)(dxl_goal_position[id]))),
                               DXL_LOBYTE(DXL_HIWORD((int)(dxl_goal_position[id]))),
                               DXL_HIBYTE(DXL_HIWORD((int)(dxl_goal_position[id])))]
        dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position)
        if not dxl_addparam_result:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            sys.exit()
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        for id in range(len(dxl_goal_position)):
            # Check if groupsyncread data of Dynamixel is available
            dxl_getdata_result = groupSyncRead.isAvailable(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if not dxl_getdata_result:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                sys.exit()

        for id in range(len(dxl_goal_position)):
            # Get Dynamixel present position value
            dxl_present_position = groupSyncRead.getData(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

            '''当前电流'''
            # dxl_present_current = groupSyncRead.getData(id, CURRENT, LEN_PRO_CURRENT)
            # print("[ID:%03d] GoalPos:%04d  PresPos:%04d PresCurrent:%04d\t" % (
            # id, dxl_goal_position[id], dxl_present_position, dxl_present_current))
            '''当前温度'''
            dxl_present_temperature = groupSyncRead.getData(id, ADDR_PRO_PRESENT_TEMPERATURE, LEN_PRO_PRESENT_TEMPERATURE)
            print(f'TEMPERATURE:{dxl_present_temperature}')

            print("[ID:%03d] GoalPos:%04d  PresPos:%04d\t" % (
                id, dxl_goal_position[id], dxl_present_position))

        f = False
        for id in range(len(dxl_goal_position)):
            dxl_present_position = groupSyncRead.getData(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

            if abs(dxl_goal_position[id] - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                f = True
                break
        if f:
            break
    # --------------------------------------------
    # current_value = []
    # for i in range(len(dxl_goal_position)):
    #     current_value.append(groupSyncRead.getData(i, CURRENT, 2))
    # print(current_value)
    groupSyncRead.clearParam()


# 结束啦,断断力矩
def gg(n):
    for id in range(n):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    portHandler.closePort()


# 用雅可比矩阵运动,传入节数,对象,当前位置,速度,你想要的时间
def move_byspeed(lenoflink, snake, q1, speed, t):
    # q1 = q1 / 180 * pi
    end2 = q1
    t = np.array(t)
    # print(t,type(t),len(t))
    dt = (t[len(t) - 1] - t[0]) / len(t)
    save = np.zeros([len(t), lenoflink])
    for i in (range(len(t))):
        j = snake.jacob0(end2)
        dq = np.dot(np.linalg.pinv(j), speed)
        end2 = end2 + np.dot(dq.T, dt)
        save[i] = end2
    sol2 = jtraj(q1, end2, t)
    # for i in range(len(sol2.q)):
    #     for k in range(len(sol2.q[i])):
    #         if sol2.q[i][k] > pi / 2:
    #             print(i, k, sol2.q[i][k])
    #             exit(-1)
    return sol2.q[:], save[:]
    # snake.plot(sol2.q).hold()


def present_pos_on_time(n):
    present_pos = []
    current_value = []
    temperature_value = []
    for i in range(n):
        present_pos.append(packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRO_PRESENT_POSITION)[0])
        current_value.append(packetHandler.read2ByteTxRx(portHandler, i, ADDR_PRO_PRESENT_CURRENT)[0])
        # current_value.append(
        #     GroupSyncRead(portHandler, packetHandler, start_address=CURRENT, data_length=2).getData(dxl_id=i,
        #                                                                                             address=CURRENT,
        #                                                                                             data_length=2))
        current_value.append(groupSyncRead.getData(i, ADDR_PRO_PRESENT_CURRENT, 2))
    # print('当前电流值：', current_value)
    return present_pos

# if __name__=='__main__':
#     lenoflink = 8
#     DHs = []
#     for i in range(int(lenoflink)):
#         DHs.append(RevoluteDH(a=1, alpha=(-1) ** (i + 1) * pi / 2))
#     snake = DHRobot(DHs, name="snake")
#     s = np.asarray([30, -60, 30, -40, 50, 50, 60, 70])
#     speed = [-0.1, -0.1, 0, 0, 0, 0]
#     t = np.arange(0, 2, 0.10)
#     print(t)
#     move_byspeed(lenoflink,snake,s,speed,t)
