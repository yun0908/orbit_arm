import csv
import msvcrt
import types
import pandas as pd
import time
import multiprocessing
import numpy as np
import time
import math
import serial
import serial.tools.list_ports
from roboticstoolbox import *
import array
import binascii
import crcmod
from time import sleep
import threading
from binascii import *
import pandas as pd
from spatialmath import SE3
import quaternion
from spatialmath.base import trotx
import arm
import orbit
from dynamixel_sdk import *
########################################################
"""滑轨部分"""
#######################################################
# 声明串口对象
ser = serial.Serial()
# 声明位置模式参数
Location_Success = "01fd026b"
# 回零返回值
Zero_Success = "019a026b"
###################################################################################
"""
                                机器臂部分
                                适配八个数个关节
"""
#################################################################################
ID = [0, 1, 2, 3, 4, 5, 6, 7]
lengths = [10, 10, 10, 10, 10, 10, 10, 10]
end_positions_list = []
rotation_matrices_list = []

ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132
ADDR_PRO_PRESENT_CURRENT = 126
ADDR_PRO_POSITION_P = 84

LEN_PRO_PRESENT_CURRENT = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = 'COM3'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
groupSyncReadcurrent = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
groupSyncWritePosition_P = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_POSITION_P, LEN_PRO_PRESENT_CURRENT)

index = 0


def read_excel(file_path):
    # 读取Excel表格数据
    df = pd.read_excel(file_path)

    # 将每行数据存储为数组，并存储到列表中
    data_list = []
    for index, row in df.iterrows():
        data_list.append(row.tolist())

    return data_list

#####################################################################################################################
if __name__ == '__main__':
    lenof_link = 8
    a_arm = 0.1
    pi = np.pi
    DHs = [RevoluteDH(a=a_arm, alpha=-pi / 2, qlim=[-70 * pi / 180, 70 * pi / 180])]
    for i in range(1, lenof_link):
        if i == lenof_link - 1:
            DHs.append(RevoluteDH(a=a_arm, alpha=0, qlim=[-75 * pi / 180, 75 * pi / 180]))
        else:
            DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** (i + 1), qlim=[-75 * pi / 180, 75 * pi / 180]))
    snake = DHRobot(DHs, name="arm")
    snake.base = snake.base @ SE3(trotx(np.pi))

    arm.check()
    orbit.GetAllSerial()  # 检查可用串口
    orbit.port_open_recv()  # 打开串口

    pos = arm.read_position(ID)
    theta_ready = [2048, 2060, 2048, 2068, 2048, 2048, 2048, 2048]

    file_path = "end.xls"               # 指定Excel文件路径
    theta_all = read_excel(file_path)   # 调用函数读取Excel表格数据
    theta_end = theta_all[0]            # 插入完成时的位置

    traj1 = arm.cal_d(pos, theta_ready, 100)
    for i in range(len(traj1)):
        arm.do(traj1[i])
        time.sleep(0.01)
    time.sleep(5)

    orbit.EleMove_Send("CW", 100, 0, 37, "rel")     #"CW"正方向 “Sep"速度 "pos" 一圈0.5cm
    if input('输入回车键进行插入操作') == '':
        print('请注意，开始执行插入。')

    params = []
    theta_all = []
    for i in range(4):
        a = 2 * (i + 1)
        theta_mid = theta_ready[:-a] + theta_end[:a]
        params.append(theta_mid)

    for i in range(len(params)):
        if i == 0:
            sol1 = arm.cal_d(theta_ready, params[0], 100)
            theta_all = theta_all + sol1
        else:
            sol2 = arm.cal_d(params[i - 1], params[i], 100)
            theta_all = theta_all + sol2

    process1 = threading.Thread(target=orbit.do2, args=("CW", 210, 0, 160, "rel"), daemon=False)
    process1.start()

    for i in theta_all:
        arm.do(i)
        time.sleep(0.15)

    sleep(1)
    ser.close()

    time.sleep(1)
    for i in range(10):
        position = arm.read_position(ID)
        print("输出当前位置：",position)

    # 指定Excel文件路径
    file_path = "C:/Users/Lenovo/Desktop/rectangle.xlsx"

    # 调用函数读取Excel表格数据
    data = read_excel(file_path)

    sol2 = []
    for i in range(len(data)):
        if i != len(data) - 1:
            sol1 = arm.cal_d(data[i], data[i + 1], 100)
            sol2 = sol2 + sol1
        else:
            break

    # 打印每行数据
    for row in sol2:
        arm.do(row)
        time.sleep(0.02)