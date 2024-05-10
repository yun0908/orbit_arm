import msvcrt
import time
import multiprocessing
import numpy as np
import time
import math
import serial
import serial.tools.list_ports
import array
import binascii
import crcmod
from time import sleep
import threading
from binascii import *
import pandas as pd
from dynamixel_sdk import *
import arm
import orbit

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
                                适配奇数个关节
"""
#################################################################################
ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132
ADDR_PRO_PRESENT_CURRENT = 126
ADDR_PRO_POSITION_P = 84

LEN_PRO_PRESENT_CURRENT = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4

PROTOCOL_VERSION = 2.0
BAUDRATE = 3000000
DEVICENAME = 'COM4'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
groupSyncReadcurrent = GroupSyncRead(portHandler, packetHandler,ADDR_PRO_PRESENT_CURRENT,LEN_PRO_PRESENT_CURRENT)
groupSyncWritePosition_P = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_POSITION_P, LEN_PRO_PRESENT_CURRENT)

index = 0


#####################################################################################################################
if __name__ == '__main__':
    arm.check()
    # CW为正方向
    orbit.GetAllSerial()  # 检查可用串口
    orbit.port_open_recv()  # 打开串口

    pos = arm.read_position(ID)

    # theta_end = [1,2,3,4,5,6,7,8,9,10,11]
    # theta_ready = [0,0,0,0,0,0,0,0,0,0,0]
    theta_ready = [2055, 2000, 2048, 2010, 2048, 2010, 2048, 2010, 2048, 2010, 0]

    #theta_end = [2048, 2000, 2048, 2000, 2048, 1800, 2048, 1700, 2048, 1600, 2048]

    theta_end = [2055.00, 2000.00, 2102.65, 1365.33, 2730.67, 2213.20, 	2496.40, 2730.67, 1365.33, 2531.70, 0]

    traj1 = arm.cal_d(pos, theta_ready, 100)
    for i in range(len(traj1)):
        arm.do(traj1[i])
        time.sleep(0.05)

    time.sleep(5)

    orbit.EleMove_Send("CW", 100, 0, 4, "rel")
    time.sleep(5)

    params = []
    for i in range(5):
        a = (i + 1) * 2 - 1
        theta_mid = theta_ready[:-a] + theta_end[2:a+2]
        params.append(theta_mid)


    for i in range(5):
        if i == 0:
            sol = arm.cal_d(theta_ready, params[i], 100)


            process1 = threading.Thread(target=orbit.do2, args=("CW", 200, 0, 20, "rel"))
            process2 = threading.Thread(target=arm.do3, args=[*sol])
            process1.start()
            process2.start()
            # 等待进程执行完毕
            process1.join()
            process2.join()


        else:
            sol = arm.cal_d(sol[-1], params[i], 100)

            process3 = threading.Thread(target=orbit.do2, args=("CW", 400, 0, 40, "rel"))
            process4 = threading.Thread(target=arm.do3, args=[*sol])
            process3.start()
            process4.start()
            # 等待进程执行完毕
            process3.join()
            process4.join()



    sleep(1)
    ser.close()


    time.sleep(1)
    for i in range(10):
        position = arm.read_position(ID)
        print("输出当前位置：",position)



    def read_excel(file_path):
        # 读取Excel表格数据
        df = pd.read_excel(file_path)

        # 将每行数据存储为数组，并存储到列表中
        data_list = []
        for index, row in df.iterrows():
            data_list.append(row.tolist())

        return data_list


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