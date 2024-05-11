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


###################################################################################
"""
                                机器臂部分
"""


#################################################################################
def getch():
    return msvcrt.getch().decode()


from dynamixel_sdk import *

ID = [0, 1, 2, 3, 4, 5, 6, 7]

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
groupSyncReadcurrent = GroupSyncRead(portHandler, packetHandler,ADDR_PRO_PRESENT_CURRENT,LEN_PRO_PRESENT_CURRENT)
groupSyncWritePosition_P = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_POSITION_P, LEN_PRO_PRESENT_CURRENT)

index = 0


def wait():
    print("go go go gogogo!!!")
    getch()


def check():
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    for i in range(len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], ADDR_PRO_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % ID[i])
    for i in range(len(ID)):
        dxl_addparam_result = groupSyncRead.addParam(ID[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % ID[i])
            quit()


# 传参为一维列表
def do(goal_position):
    DXL_goal_position = []
    for i in range(len(ID)):
        DXL_goal_position = [DXL_LOBYTE((DXL_LOWORD(int(goal_position[i])))),
                             DXL_HIBYTE((DXL_LOWORD(int(goal_position[i])))),
                             DXL_LOBYTE((DXL_HIWORD(int(goal_position[i])))),
                             DXL_HIBYTE((DXL_HIWORD(int(goal_position[i]))))]

        # 写入位置信息的语句 执行到此 电机开始转动
        dxl_addparam_result = groupSyncWrite.addParam(ID[i], DXL_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % ID[i])
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()


def bye():
    for i in range(len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], ADDR_PRO_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    portHandler.closePort()


# 分割函数（速度控制函数） 将两个姿态分割为n份
def cal_d(pos1, pos2, n):
    # n为想要分割的数量, pos2为目标构型，pos1为起始构型
    pos = [[i for i in range(len(pos1))] for j in range(n + 1)]
    for i in range(n + 1):
        for j in range(len(pos1)):
            D = (pos2[j] - pos1[j])
            pos[i][j] = int(pos1[j] + (i * (D / n)))
    return pos


def read_position(ID):
    pre_pos = []
    for i in range(len(ID)):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID[i],
                                                                                       ADDR_PRO_PRESENT_POSITION)
        pre_pos.append(dxl_present_position % 4095)
    return pre_pos

 # 获取电机电流
def get_currents(ID):
    currents = []
    for i in range(len(ID)):
        dxl_currents, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID[i],
                                                                               ADDR_PRO_PRESENT_CURRENT)

        if dxl_currents > 2 ** 15:
            dxl_currents -= 2 ** 16
        currents.append(dxl_currents)

    return currents

def do3(*sol):
    for i in sol:
        do(i)
        time.sleep(0.05)

def main():
    print("arm over")

if __name__=="__main__":
    main()