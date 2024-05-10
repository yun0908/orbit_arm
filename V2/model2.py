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

########################################################
"""滑轨部分"""
#######################################################

# 声明串口对象
ser = serial.Serial()
# 声明位置模式参数
Location_Success = "01fd026b"
# 回零返回值
Zero_Success = "019a026b"


def port_open_recv():  # 对串口的参数进行配置
    ser.port = 'COM5'
    ser.baudrate = 115200  # 波特率
    ser.bytesize = 8
    ser.stopbits = 1
    ser.parity = "N"  # 奇偶校验位
    ser.open()
    if (ser.isOpen()):
        print("串口打开成功！")
    else:
        print("串口打开失败！")


# isOpen()函数来查看串口的开闭状态
def port_close():
    ser.close()
    if (ser.isOpen()):
        print("串口关闭失败！")
    else:
        print("串口关闭成功！")


def send(send_data):
    if (ser.isOpen()):
        ser.write(send_data.encode('utf-8'))  # 编码
        print("发送成功", send_data)
    else:
        print("发送失败！")


def GetAllSerial():
    # 获取所有串口设备实例。
    # 如果没找到串口设备，则输出：“无串口设备。”
    # 如果找到串口设备，则依次输出每个设备对应的串口号和描述信息。
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])


# CCR校验函数
# 十进制转十六进制
def Int2hex(num):
    return hex(int(num, 10))


# 十六进制转十进制
def hex2Int(num):
    return int(num, 16)


# 输入绝对位置参数
def ElePosit(Posit):
    return '{:08X}'.format(Posit)


# 让电机以位置模式转多少圈：计算圈数
def EleRoop(loop):
    #  print("位置的参数为" + '{:08X}'.format(loop * 3200))
    return '{:08X}'.format(loop * 3200)


# 获取电机转动速度
def GetEleSped(Speed):
    #  print("转动速度的参数为" + '{:04X}'.format(Speed))
    return '{:04X}'.format(Speed)


# 获取电机加速度（默认为0？）
def GetEleAcc(Acc):
    # print("加速度的参数为" + '{:02X}'.format(Acc))
    return '{:02X}'.format(Acc)


# 电机返回角度的计算函数
# 输入十六进制返回角度值
def CalEleAngle(Angle):
    # 十六进制转十进制
    IntAngle = hex2Int(Angle)
    return (IntAngle * 360) / 65536


# 驱动电机转动,输入电机转向，（地址暂时不用），速度(上限是5000)和加速度，电机转动的圈数（转动的位置）(转动多少圈)，位置模式 rel 代表相对，abs 代表绝对，返回转动的报文
# 转向： 00 CW 01 CCW
# CW 向前
def EleMove(Tun, Sep, Acc, Pos, PosEle):
    # 添加地址
    SendAdd = "01"  # 地址
    # 添加固定格式报文
    SendAdd = SendAdd + "FD"
    # 添加转向
    if Tun == "CW":
        SendAdd = SendAdd + "00"
    if Tun == "CCW":
        SendAdd = SendAdd + "01"
    # 添加速度报文
    SendAdd = SendAdd + GetEleSped(Sep)
    # 添加加速度报文
    SendAdd = SendAdd + GetEleAcc(Acc)
    # 添加脉冲数报文
    SendAdd = SendAdd + EleRoop(Pos)
    # 添加位置模式报文
    if PosEle == "rel":
        SendAdd = SendAdd + "00"
    if PosEle == "abs":
        SendAdd = SendAdd + "01"
    # 添加是否多机同步（默认不启用）
    SendAdd = SendAdd + "00"
    # 置入CCR校验字节(默认最后一个字节是0x6B)
    SendAdd = SendAdd + "6B"
    return SendAdd


# 输入对应的电机转向，（地址暂时不用），速度和加速度，脉冲数（转动的位置）(转动多少圈)，位置模式 rel 代表相对，abs 代表绝对，输出转动的报文指令
# 转向： 00 CW 01 CCW

# 读取数据的函数,设定为默认接收位置模式返回的讯息
def EleRead_Location() -> bool:
    data = ser.read(4)  # 读四个字节指令
    data_hex = str(binascii.b2a_hex(data))[2:-1]
    # print(data_hex)
    f = data_hex == Location_Success
    if f:
        print("数据发送成功")

    else:
        print("数据发送失败")
    ser.flushInput()  # 清除输入缓存区数据
    return f


# 驱动电机转动,输入电机转向，（地址暂时不用），速度和加速度，脉冲数（转动的位置）(转动多少圈)，位置模式 rel 代表相对，abs 代表绝对，返回转动的报文
# 转向： 00 CW 01 CCW
# 添加自动指令发送功能
def EleMove_Send(Tun, Sep, Acc, Pos, PosEle):
    # 添加地址
    SendAdd = "01"  # 地址
    # 添加固定格式报文
    SendAdd = SendAdd + "FD"
    # 添加转向
    if Tun == "CW":
        SendAdd = SendAdd + "00"
    if Tun == "CCW":
        SendAdd = SendAdd + "01"
    # 添加速度报文
    SendAdd = SendAdd + GetEleSped(Sep)
    # 添加加速度报文
    SendAdd = SendAdd + GetEleAcc(Acc)
    # 添加脉冲数报文
    SendAdd = SendAdd + EleRoop(Pos)
    # 添加位置模式报文
    if PosEle == "rel":
        SendAdd = SendAdd + "00"
    if PosEle == "abs":
        SendAdd = SendAdd + "01"
    # 添加是否多机同步（默认不启用）
    SendAdd = SendAdd + "00"
    # 置入CCR校验字节(默认最后一个字节是0x6B)
    SendAdd = SendAdd + "6B"
    #  print(SendAdd)
    ser.write(bytes.fromhex(SendAdd))
    # 读取电机返回指令，观察指令是否发送成功
    # while not EleRead_Location():
    #
    # sleep(0.01)


# 读取数据的函数,设定为默认接收位置模式返回的讯息
# 电机回零指令接收校验
def EleRead_Zero():
    data = ser.read(4)  # 读四个字节指令
    data_hex = str(binascii.b2a_hex(data))[2:-1]
    # print(data_hex)
    if data_hex == Zero_Success:
        print("数据发送成功")
    else:
        print("数据发送失败")
    ser.flushInput()  # 清除输入缓存区数据


# 电机位置角度清零
def Location_TunZero():
    data = "010a6d6b"
    ser.write(bytes.fromhex(data))


# 电机回零指令发送
# 触发回零：默认为多圈无限位碰撞回零

def EleTurnZero():
    SendData = "01"  # 添加地址
    SendData = SendData + "9a"  # 添加固定标志位
    # 添加触发回零模式
    SendData = SendData + "02"
    SendData = SendData + "006b"  # 多机控制位和校验位
    # 发送数据
    ser.write(bytes.fromhex(SendData))
    EleRead_Zero()
    Location_TunZero()
    ser.flushInput()  # 清除输入缓存区数据


# 电机立即停止指令
def EleStop_initially():
    data = "01fe98006b"
    ser.write(bytes.fromhex(data))


# 读取电机当前位置的函数，当前读取的是相对位置
def EleAbsolute_Read():
    data = "01366b"
    ser.write(bytes.fromhex(data))

    data = ser.read(8)  # 读八个字节指令
    data_hex = str(binascii.b2a_hex(data))[2:-1]
    #  print(data_hex)
    isfu = data_hex[4:6]  # 是否为负数
    #  print(isfu)
    Location_Current = data_hex[6:14]  # 当前角度
    # print(Location_Current)
    Location = str(CalEleAngle(Location_Current))  # 计算角度

    if (isfu == "00"):
        return "+" + Location
    else:
        return "-" + Location


def do2(Tun, Sep, Acc, Pos, PosEle):
    EleMove_Send(Tun, Sep, Acc, Pos, PosEle)
    # EleTurnZero()


###################################################################################
"""
                                机器臂部分
"""


#################################################################################
def getch():
    return msvcrt.getch().decode()


from dynamixel_sdk import *

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


# 获取电机位置
def read_position(ID):
    pre_pos = []
    for i in range(len(ID)):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID[i],
                                                                                       ADDR_PRO_PRESENT_POSITION)
        pre_pos.append(dxl_present_position % 4095)
    return pre_pos


# 获取末端旋转矩阵和坐标
def calculate_end_position(joint_angles1, joint_lengths):
    def transform_value(x):
        return x * 90 / 1024

    joint_angles = [transform_value(x) for x in joint_angles1]

    def convert_to_dh_params(joint_angles, joint_lengths):
        dh_params = []
        for i in range(len(joint_angles)):
            alpha = 0
            a = 0
            d = joint_lengths[i]
            theta = joint_angles[i]
            dh_params.append([alpha, a, d, theta])
        return np.array(dh_params)

    def dh_rotation_matrix(alpha, a, d, theta):
        alpha = np.radians(alpha)
        a = a
        d = d
        theta = np.radians(theta)

        rot_alpha = np.array([
            [1, 0, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha), 0],
            [0, np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 0, 1]
        ])

        trans_a = np.array([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        trans_d = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

        rot_theta = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return rot_alpha @ trans_a @ trans_d @ rot_theta

    dh_params = convert_to_dh_params(joint_angles, joint_lengths)

    T = np.identity(4)

    for params in dh_params:
        T = np.dot(T, dh_rotation_matrix(*params))

    end_position = T[:3, 3]
    rotation_matrix_end = T[:4, :4]

    return end_position, rotation_matrix_end  # end_position：末端坐标  rotation_matrix_end  末端旋转矩阵


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
        # process1 = threading.Thread(target=do2, args=("CW", 300, 0, 1, "rel"), daemon=False)
        # process2 = threading.Thread(target=do, args=[i], daemon=False)
        do(i)
        # process1.start()
        # process2.start()
        # process1.join()
        # process2.join()
        # while process2.is_alive():  # process1.is_alive()
        #     pass
        # bujin = EleAbsolute_Read()
        # angles = read_position(ID)
        # angles = [(i - 2048) / 4096 * 2 * np.pi for i in angles]
        # end_position, rotation_matrix_end = calculate_end_position(angles, lengths)
        # end_positions_list.append(end_position)   #末端坐标
        # rotation_matrices_list.append(rotation_matrix_end)   #末端旋转矩阵
        # Tt = snake.fkine(angles)
        # Tt_.append(Tt)
        # bujin = EleAbsolute_Read()
        # print(bujin)

    # 保存为Excel文件
    # Dd = pd.DataFrame([Tt_],[bujin_])
    # Dd.to_excel('data.xlsx', index=False)
    # print(Tt)


def read_excel(file_path):
        # 读取Excel表格数据
        df = pd.read_excel(file_path)

        # 将每行数据存储为数组，并存储到列表中
        data_list = []
        for index, row in df.iterrows():
            data_list.append(row.tolist())

        return data_list
pi = np.pi


if __name__ == '__main__':
    theta_ready = [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
    # 指定Excel文件路径
    file_path = "end.xls"
    # 调用函数读取Excel表格数据
    theta_all = read_excel(file_path)
    theta_end = theta_all[0]


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

    theta_test = [(i - 2048) / 4096 * 2 * np.pi for i in theta_end]
    snake.teach(theta_test)
    exit()
    params_mod = []
    poses = []
    sol = []
    angle2 = []
    file = open('example.csv', 'w', newline='')
    for i in range(4):
        a = 2 * (i + 1)
        theta_mid = theta_ready[:-a] + theta_end[:a]
        params_mod.append(theta_mid)

    for i in range(4):
        if i == 0:
            sol = cal_d(theta_ready, params_mod[i], 50)
            angles = sol

        else:
            sol = cal_d(sol[-1], params_mod[i], 50)
            angles =angles+sol

    p=0
    for i in angles:

        angle1=[]
        for ii in i:
            # print(ii,type(ii))
            # if type(ii)==type([]):
            #     print("get list:{}-{}".format(i,ii))
            #     exit()
            angle1.append((ii - 2048) / 4096 * 2 * np.pi)

        # print(angle1)
        # exit()
        a=snake.fkine(angle1)
        q = quaternion.from_rotation_matrix(a.R)
        sol.append(np.array(a))
        # if type(q)!=quaternion.quaternion:
        #     print(q)
        #     exit()
        writer=csv.writer(file)
        writer.writerow([q.x,q.y,q.z,q.w,a.t[0]+p,a.t[1],a.t[2]])
        p+=1

    













