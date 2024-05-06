# -*- coding: utf-8 -*-
# from ast import Pass
# import profile
# from re import X
# import sys
# import os
import time
import math
# from tkinter import CURRENT
# from wsgiref import validate
# from spatialmath import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


class ikine:
    def __init__(self, r):
        self.r = r

    def unitstep(self, x):
        return 1 if x >= 0 else 0

    def dpoint(self, xd, yd):
        # return math.acos((-2*self.r**2 + xd**2 + yd**2)/(2*self.r*math.sqrt(xd**2 + yd**2))) - math.acos((-self.r**2+self.r**2*(3-2*math.sqrt(2)*math.cos(math.acos((4*self.r**2-xd**2-yd**2)/(2*math.sqrt(3)*self.r**2))+math.pi/12))/(2*self.r**2*math.sqrt(3-2*math.sqrt(2)*math.cos(math.acos((4*self.r**2-xd**2-yd**2)/(2*math.sqrt(3)*self.r**2))+math.pi/12)))) - math.acos((self.r**2*(3-2*math.sqrt(2)*math.cos(math.acos((4*self.r**2-xd**2-yd**2)/(2*math.sqrt(3)*self.r**2))+math.pi/2))-self.r**2)/(2*self.r**2*math.sqrt(3-2*math.sqrt(2)*math.cos(math.acos((4*self.r**2-xd**2-yd**2)/(2*math.sqrt(3)*self.r**2))+math.pi/12))))) + 2*(self.unitstep(yd)-1/2)*math.acos(xd/(math.sqrt(xd**2+yd**2))) - 2*math.pi*(self.unitstep(yd)-1)
        edgeO1D = math.sqrt(xd ** 2 + yd ** 2)
        angXO1D = 2.0 * (self.unitstep(yd) - 1 / 2.0) * math.acos(xd / edgeO1D) - 2.0 * math.pi * (
                self.unitstep(yd) - 1)
        # print('edgeO1D', edgeO1D)
        angDO1B = math.acos((edgeO1D ** 2 - 2.0 * self.r ** 2) / (2.0 * self.r * edgeO1D))
        th2 = angXO1D + angDO1B
        th3 = math.acos((4.0 * self.r ** 2 - edgeO1D ** 2) / (2 * math.sqrt(3.0) * self.r ** 2))
        angO1BC = th3 + math.pi / 12.0
        edgeO1C = self.r * math.sqrt(3.0 - 2.0 * math.sqrt(2.0) * math.cos(angO1BC))
        angBO1C = math.acos((edgeO1C ** 2 - self.r ** 2) / (2.0 * self.r * edgeO1C))

        temp = (edgeO1C ** 2 - self.r ** 2 - math.sqrt(3.0) * self.r ** 2) / (2.0 * self.r * edgeO1C)
        # print(temp)
        if abs(temp) > 1:
            if abs(temp) - 1 > 0.01:
                print('\n\n\n\n\n\n\n\nerror')
            print(temp)
            temp = int(temp)
        angAO1C = math.acos(temp)
        angAO1B = angAO1C + angBO1C
        # th1 = th2 - angAO1B
        # 考虑到实际初始化的�?�?��要使最终的th1加上2pi
        th1 = th2 - angAO1B + 2 * math.pi
        # 若初始化时的零位�?��垂直向上，则在使用时要再减去pi/2
        return th1, th2

    def cpoint(self, xc, yc):

        # 考虑到实际初始化的�?�?��要使最终的th1加上2pi
        edgeO1C = math.sqrt(xc ** 2 + yc ** 2)
        angXO1C = 2 * (self.unitstep(yc) - 1 / 2.0) * math.acos(xc / edgeO1C) - 2 * math.pi * (self.unitstep(yc) - 1)
        angBO1C = math.acos((edgeO1C ** 2 - self.r ** 2) / (2 * self.r * edgeO1C))
        th2 = angXO1C + angBO1C
        angAO1C = math.acos((edgeO1C ** 2 - (math.sqrt(3) + 1) * self.r ** 2) / (2 * self.r * edgeO1C))
        th1 = angXO1C - angAO1C + 2 * math.pi
        # 若初始化时的零位�?��垂直向上，则在使用时要再减去pi/2
        return th1, th2


class fkine:
    #   !!我忘了到底是�?��角大于哪�?��
    #   ！！beta大于pi时后面的式子也�?讨�?，�?在mma里再�?
    def __init__(self, r):
        self.r = r
        self.d1 = math.sqrt(2) * r
        self.d2 = math.sqrt(2 + math.sqrt(3)) * r

    def dpoint(self, q1, q2):
        r = self.r
        d1 = self.d1
        d2 = self.d2
        beta = q2 - q1
        AB_len = 2 * r * math.sin(beta / 2)
        BAC_ang = math.acos((AB_len ** 2 + d2 ** 2 - d1 ** 2) / (2 * AB_len * d2))

        delta = 0
        if beta <= math.pi:
            delta = q1 - math.acos(math.sqrt((1 - math.cos(beta)) / 2)) - BAC_ang + math.pi
        elif beta > math.pi:
            delta = q1 + math.acos(math.sqrt((1 - math.cos(beta)) / 2)) - BAC_ang + math.pi
        else:
            print("check fkine.jacobe.beta again")

        xc = r * math.cos(q1) + d2 * math.cos(delta)
        yc = r * math.sin(q1) + d2 * math.sin(delta)

        xb = r * math.cos(q2)
        yb = r * math.sin(q2)

        BC_vec = np.transpose(SE2(xc - xb, yc - yb).t)  # �?��后变成列向量

        theta4 = -math.pi / 12
        R = SE2(0, 0, theta4)
        d = R.R @ BC_vec
        xd = d[0]
        yd = d[1]

        return xd, yd


class rhext3:
    # Control table address
    ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION = 116
    ADDR_PRO_PRESENT_POSITION = 132
    ADDR_PRO_GOAL_VELOCITY = 104
    ADDR_PRO_PRESENT_VELOCITY = 128
    ADDR_PRO_PROFILE_VELOCITY = 112
    # ADDR_PRO_GOAL_CURRENT = 
    ADDR_PRO_PRESENT_CURRENT = 126
    ADDR_PRO_HOMING_OFFSET = 20
    ADDR_PRO_PRESENT_VOLTAGE = 144

    # Data Byte Length
    LEN_PRO_GOAL_POSITION = 4
    LEN_PRO_PRESENT_POSITION = 4
    LEN_ADDR_PRO_GOAL_VELOCITY = 4
    LEN_PRO_PRESENT_VELOCITY = 4
    LEN_PRO_PRESENT_CURRENT = 2
    LEN_PRO_HOMING_OFFSET = 4
    LEN_PRO_PRESENT_VOLTAGE = 2
    LEN_ADDR_PRO_PROFILE_VELOCITY = 4

    # Protocol version
    PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

    # BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
    BAUDRATE = 3000000  # Dynamixel default baudrate : 57600
    DEVICENAME = 'COM16'  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE = 1  # Value for enabling the torque
    TORQUE_DISABLE = 0  # Value for disabling the torque
    DXL_MINIMUM_POSITION_VALUE = 100  # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE = 4000  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold
    # DXL_MOVING_STATUS_THRESHOLD = 1  # Dyn
    # amixel moving status threshold

    # mode
    ADDR_OPERATING_MODE = 11
    EXT_POSITION_CONTROL_MODE = 4
    VELOCITY_MODE = 1
    POSITION_MODE = 3
    CURRENT_BASED_POSITION_CONTROL_MODE = 5

    index = 0
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

    def __init__(self, r, gear_rate):
        # 我的参数
        # 三�?形定义：那一侧上�?��，就说是那一侧三角形
        # id_list = [0, 1]
        self.id_list = [i for i in range(12)]
        self.id_list_left = [2, 3, 6, 7, 10, 11]
        self.id_list_right = [0, 1, 4, 5, 8, 9]
        self.dt = 0.04

        # 机器人的模式
        self.wheel_mode = 0
        self.rhex_mode = 1
        self.leg_mode = 2

        # 腿模式行走的一些参数
        self.walk_len = 0.06
        self.step_H = -2.0 * r  # 腿末端到转轴的垂直高度
        self.step_h = 0.6 * r  # 腿行走时抬起的高度
        self.walk_vel = 0.04

        self.gear_rate = gear_rate
        self.r = r
        self.ikine = ikine(r)

        # self.curmove = 'init' # 这样改不好，因为如果一�?��令�?一�?��令，会�?覆盖
        self.curmotormode = 'extend_position'
        self.currobotmode = 'init'

        '''
        # 创建处理数据传输的�?�?
        '''
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            # getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            # getch()
            quit()

    '''
    # 开关力�?
        输入：若干电机编号（列表）；开或关信号（字符串，enable、disable�?
        功能：开关电机的力矩
    '''

    def switch_torque(self, id_list, switch):
    #控制力矩
        if switch == 'enable':
            for id in id_list:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                               self.ADDR_PRO_TORQUE_ENABLE,
                                                                               self.TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Dynamixel#%d has been successfully connected" % id)
        elif switch == 'disable':
            for id in id_list:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                               self.ADDR_PRO_TORQUE_ENABLE,
                                                                               self.TORQUE_DISABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("parameters of switch_torque wrong!")

        return

    '''
    # 切换电机的模�?
        输入：若干电机编号（列表）；模式名字（字符串，三种模式：position、extend_position、velocity�?
        注意：切换模式前要先把电机力矩关掉才能换模式。结束后再打开力矩�?
    '''

    def switch_mode(self, id_list, mode_name):

        self.switch_torque(id_list, 'disable')

        mode = self.EXT_POSITION_CONTROL_MODE
        if mode_name == 'position':
            mode = self.POSITION_MODE
        elif mode_name == 'extend_position':
            mode = self.EXT_POSITION_CONTROL_MODE
        elif mode_name == 'velocity':
            mode = self.VELOCITY_MODE
        elif mode == 'current-base position':
            mode = self.CURRENT_BASED_POSITION_CONTROL_MODE

        for id in id_list:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                           self.ADDR_OPERATING_MODE,
                                                                           mode)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode changed to " + mode_name + " control mode.")

        self.switch_torque(id_list, 'enable')

        return

    '''
    # 初始化电机，把所有电机换到位置模式
    '''

    def init(self):

        self.switch_torque(self.id_list, "disable")

        for id in self.id_list:
            # Set operating mode to extended position control mode
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                           self.ADDR_OPERATING_MODE,
                                                                           self.EXT_POSITION_CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode changed to extend_position control mode.")

        self.switch_torque(self.id_list, "enable")

        # set profile velocitys
        '''
        pro_vel = 800
        profile_vel = []
        for i in range(len(self.id_list)):
            profile_vel.append([DXL_LOBYTE(DXL_LOWORD(pro_vel)),
                                DXL_HIBYTE(DXL_LOWORD(pro_vel)),
                                DXL_LOBYTE(DXL_HIWORD(pro_vel)),
                                DXL_HIBYTE(DXL_HIWORD(pro_vel))])
        
        self.write_data(self.id_list, self.ADDR_PRO_PROFILE_VELOCITY, self.LEN_ADDR_PRO_PROFILE_VELOCITY, profile_vel)
        # print(profile_vel)
        '''



    '''
    # 同�?读写电机的存储器上的数据
        输入：若干电机编号（列表）；要�?写的位置的地址（标量，预定义）；�?读写的位�?��地址长度（标量，预定义）
        输出：各�?��机内对应地址的值（列表�?/�?
    '''

    def read_data(self, id_list, adr, adr_len):
        # 造卡�?
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, adr, adr_len)

        # 告诉卡车有哪些�?货站
        for id in id_list:
            dxl_addparam_result = groupSyncRead.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)
                quit()

        # 卡车出发
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # 看各站是否有�?
        for id in id_list:
            dxl_getdata_result = groupSyncRead.isAvailable(id, adr, adr_len)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                # quit()
                # sys.exit()

        # 卡车卸货，�?取数�?
        data = []
        for id in id_list:
            data.append(groupSyncRead.getData(id, adr, adr_len))

        # 因为�?��在函数里，所以卡车是局部变量，出去之后�?��销毁，不用清空�?

        return data

    def write_data(self, id_list, adr, adr_len, data):
        # 造卡�?
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, adr, adr_len)

        # 告诉卡车有哪些�?货站
        for i in range(len(id_list)):
            dxl_addparam_result = groupSyncWrite.addParam(id_list[i], data[i])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % i)
                # quit()

        # 卡车出发到各站，并卸�?
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # 因为�?��在函数里，所以卡车是局部变量，出去之后�?��销毁，不用清空�?

        return

    '''
    # # 读取电机的位�?�?
    #     输入：若干电机的编号（列�?��
    #     输出：若干电机的位置（列�?��元素类型是int）�?2pi划分�?4096份。position模式下的范围�?0�?4095，extend_position和velocity模式下的范围�?0～很大的一�?�?
    #
    #     # 读取�?��外�?应id号的杆子的位�?�?
    #     考虑到电机和杆子�?��有齿�?��要考虑齿轮的加速减速比，�?果isout为True，那么输入就指的�?��机连杆的数据
    #         输入：若干杆子�?应电机的id号（列表�?
    #         输出：每�?��子�?应的角度（列�?��元素类型是float，单位是电机制，所有有关�?度的都转化成电机制）
    # '''

    # 获取电机位置
    def get_positions(self, id_list):
        positions = self.read_data(id_list, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

        # 这里�?��了解决溢出问�?
        for i in range(len(positions)):
            if positions[i] > 2 ** 31:
                positions[i] -= 2 ** 32
        return positions

    # 获取电机速度
    def get_velocitys(self, id_list):
        velocitys = self.read_data(id_list, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_PRESENT_VELOCITY)

        # 这里�?��了解决溢出问�?
        for i in range(len(velocitys)):
            if velocitys[i] > 2 ** 31:
                velocitys[i] -= 2 ** 32
        return velocitys

    # 获取电机电流
    def get_currents(self, id_list):
        currents = self.read_data(id_list, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

        # 这里�?��了解决溢出问�?
        for i in range(len(currents)):
            if currents[i] > 2 ** 15:
                currents[i] -= 2 ** 16
        return currents

    # 获取电机电压
    def get_voltage(self, id_list):
        voltage = self.read_data(id_list, self.ADDR_PRO_PRESENT_VOLTAGE, self.LEN_PRO_PRESENT_VOLTAGE)

        for i in range(len(voltage)):
            if voltage[i] > 2 ** 15:
                voltage[i] = 2 ** 16 - voltage[i]
        return voltage

    def print_allall(self, id_list, t0):
        # 读的都是电机的�?
        print('delta_t:\t', time.time() - t0)
        print('cur_pos:\t', self.get_positions(id_list))
        print('cur_vel:\t', self.get_velocitys(id_list))
        print('cur_cur:\t', self.get_currents(id_list))

        return

    '''
    # 设置每个电机的位�?��指定�?��
        输入：若干电机的编号（列�?��；各�?��机�?到的�?��点（列表，输入的�?��位都�?��机制�?;�?���??定杆子�?速度
        功能：�?各给定电机以默�?速度�?4 rad/s; 167 * 0.227 rpm）转到各指定�?���?
    
    
        输入：若干杆子�?应电机的id号（列表）；杆子想到达的位置（列�?��单位�?��机制�?
        功能：�?各给定电机以默�?速度�?4 rad/s; 167 * 0.227 rpm）转到各指定位置的两倍�?
    '''

    def set_positions(self, id_list, goal_positions):
        for i in range(len(goal_positions)):
            goal_positions[i] = int(round(goal_positions[i]))

        param_goal_positions = []
        for i in range(len(id_list)):
            param_goal_positions.append([DXL_LOBYTE(DXL_LOWORD(goal_positions[i])),
                                         DXL_HIBYTE(DXL_LOWORD(goal_positions[i])),
                                         DXL_LOBYTE(DXL_HIWORD(goal_positions[i])),
                                         DXL_HIBYTE(DXL_HIWORD(goal_positions[i]))])
        self.write_data(id_list, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION, param_goal_positions)
        return

    '''
    # 给定电机在各�?��前位�?��相同角速度往前转
        输入：若干电机的编号（列�?��；�?速度（标量，弧度制）；转动时间（标量，单位是秒）
        过程�?
            把�?速度�?��成电机制，算出�?达到�?��角速度应�?每�?指令让它多转多少dposition（发指令的间隔在预定义）
            读取当前位置，先算出各个时刻应�?到达�?��位置偏量
            用while�?��当前距�?开始时间的时间间隔，判�?��没有到达指定�?��时间，到达就退出�?
        注意�?
            实际的轮式模式、rhex模式应�?采用速度模式，这样没有转的圈数的限制
            执�?这�?代码时�?果电机出现抖�?���?���?��为数�?��之间离得�?��，有电�?干扰
            �?��作库函数，有while�?��
        如果输入的速度指的�??面的速度
            输入的isout就为True，那么内部�?乘上gear_rate
            但是读的还是电机的位�?��在电机当前位�?��基�?上，速度为�?定的gear_rate倍，这样�?
    '''

    def roll_pos_control(self, id_list, vel_th, durt):

        t_start = time.time()
        print('==========')
        print('>>>>> roll_pos_control() Start!')

        dposition_raw = vel_th * self.dt * 4096 / (2 * math.pi)
        # dposition = int(dposition_raw)

        data = self.get_positions(id_list)

        t0 = time.time()
        t = 0
        N = 0
        read_data_t = []
        read_data_pos = []
        read_data_vel = []
        read_data_cur = []
        while t < durt:
            t = time.time() - t0
            if t > N * self.dt:
                goal_positions = []
                for i in range(len(data)):
                    goal_positions.append(data[i] + N * dposition_raw)  # 注意这里int;不用注意了，我写在函数里了，需要四舍五入的int
                self.set_positions(id_list, goal_positions)
                # nowpos = self.get_positions(id_list)
                # print('t: ', t, '\t', nowpos, goal_positions)

                # 读的都是电机的�?
                read_t = time.time() - t0
                read_pos = self.get_positions(id_list)
                read_vel = self.get_velocitys(id_list)
                read_cur = self.get_currents(id_list)
                print('delta_t:\t', read_t)
                print('cur_pos:\t', read_pos)
                print('cur_vel:\t', read_vel)
                print('cur_cur:\t', read_cur)
                read_data_t.append(read_t)
                read_data_pos.append(read_pos)
                read_data_vel.append(read_vel)
                read_data_cur.append(read_cur)

                N = N + 1

        print('^^^^^ roll_pos_control() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        return read_data_t, read_data_pos, read_data_vel, read_data_cur

    def roll_pos_control_V2(self, id_list, vel_th, durt):

        t_start = time.time()

        print('====== roll start ======')

        dposition = vel_th * self.dt * 2048 / math.pi

        positions = self.get_positions(id_list)

        goal_positions = []

        t0 = time.time()
        t = 0
        N = 0

        while t < durt:
            t = time.time() - t0
            if t > N * self.dt:
                goal_positions = [int(position + N * dposition) for position in positions]
                print('present position : ', positions)
                print('goal positions', goal_positions)
                self.set_positions(id_list, goal_positions)

                N = N + 1

        print("roll finish!")
        print("time consume:", time.time() - t_start)

    '''
    （弃�?��判断�?��所有点都到达目标点
        输入：若干电机的编号（列�?��；各电机各自要到达都�?��点；传入的t0（用于输出时间）
        输出：给出的电机�?��都各�?��达了指定的目标点，都到了输出True，有一�?��到就输出False
        注意�?
            在实际测试中发现，调用这�?��数会有延迟，写在while的判�?��会�?致跳不出�?
            所以这段代码作为模版，要用的时候�?制进去根�?��况改
    '''

    def judge_isarrived_target_position(self, id_list, target_positions, t0):
        positions = self.get_positions(id_list)
        print('t: ', time.time() - t0, '\t', positions, target_positions)
        all_arrived = True
        for i in range(len(id_list)):
            if abs(positions[i] - target_positions[i]) > self.DXL_MOVING_STATUS_THRESHOLD:
                all_arrived = False

        return all_arrived

    '''
    # 给所有电机�?�?��一相位
        输入：若干电机的编号（列�?��；指定的一�?���?��标量，电机制�?
        功能：�?所有给出的电机都以默�?速度�?��同一�?���?
        注意：是动作库函数，有while�?��
    
        如果isout为True，那么输入就指的�?��机连杆的数据
    '''

    def set_same_target_position(self, id_list, same_target_position):

        t0 = time.time()
        all_arrived = False
        while 1:
            target_positions = [same_target_position] * len(id_list)
            all_arrived = self.judge_isarrived_target_position(id_list, target_positions, t0)

            if not all_arrived:
                goal_position = []
                for i in range(len(id_list)):
                    goal_position.append(same_target_position)
                self.set_positions(id_list, goal_position)
            else:
                break
        return

    '''
    # 让所有给出电机以默�?速度各自到达�?��位置
        输入：若干电机的编号（列�?��；各电机各自要到达都位置（列�?��单位�?��机制度）
        注意：是动作库函数，有while�?��
    
        如果isout为True，那么输入就指的�?��机连杆的数据
    '''

    def set_allarrive_target_position(self, id_list, target_positions):

        t0 = time.time()
        all_arrived = False
        while 1:
            positions = self.get_positions(id_list)
            # print('t: ', time.time() - t0, '\t', positions, target_positions)
            # self.print_allall(id_list, t0)
            all_arrived = True
            for i in range(len(id_list)):
                if abs(positions[i] - target_positions[i]) > self.DXL_MOVING_STATUS_THRESHOLD:
                    all_arrived = False

            if not all_arrived:
                self.set_positions(id_list, target_positions)
            else:
                break
        return

    """
    使电机在一定时间内�?��指定为位�?
    """

    def set_allarrive_target_position_V2(self, id_list, target_positions, durt):
        positions = self.get_positions(id_list)
        move_positions = []
        for i in range(len(positions)):
            move_positions.append(target_positions[i] - positions[i])
        # 根据�?��时间求出�?��速度
        vel = []
        for i in range(len(positions)):
            vel.append((move_positions[i] / 2048 * math.pi) / durt)
        # 根据�?��时间和速度求出每条指令�?��的�?�?
        dposition = []
        for i in range(len(positions)):
            dposition.append(vel[i] * self.dt * 2048 / math.pi)

        t0 = time.time()
        N = 0
        goal_positions = [0] * len(id_list)
        all_arrived = False
        while time.time() - t0 < durt:
            t = time.time() - t0
            if t > N * self.dt:
                for i in range(len(positions)):
                    goal_positions[i] = round(positions[i] + N * dposition[i])
                # print('goal position:', goal_positions)
                self.set_positions(id_list, goal_positions)

                for i in range(len(positions)):
                    if abs(goal_positions[i] - target_positions[i]) < self.DXL_MOVING_STATUS_THRESHOLD:
                        all_arrived = True
                    else:
                        pass
                # 如果电机达到�?��位置则退出循�?
                if all_arrived:
                    break
                N += 1

                # read_cur = self.get_currents(id_list)
                # print('cur_cur:\t', read_cur)

    '''
    # 给指定电机各�??�?��度
        输入：若干电机的编号（列�?��；各电机各自要达到都速度（列�?��
        注意�?
            调用这个函数前，必须要在速度模式下。在速度模式下才能直接赋值速度�?
            如果设置完速度没有关闭力矩（比如异常退出程序的情况），电机会继�?��当前速度�?��
    
        如果isout为True，那么输入就指的�?��机连杆的数据
    '''

    def set_velocitys(self, id_list, goal_velocitys):
        # switch_mode(id_list, 'velocity') 要用这个函数，就要在那个模块上加上这�?
        # 这个函数不能单独使用，因为�?考虑到模式转换的�??
        goal_velocitys_trans = []
        for i in range(len(goal_velocitys)):
            goal_velocitys_trans.append(int(round(goal_velocitys[i] * 60 / (2 * math.pi * 0.229))))
        # print(goal_velocitys_trans)
        param_goal_velocitys = []
        for i in range(len(id_list)):
            param_goal_velocitys.append([DXL_LOBYTE(DXL_LOWORD(goal_velocitys_trans[i])),
                                         DXL_HIBYTE(DXL_LOWORD(goal_velocitys_trans[i])),
                                         DXL_LOBYTE(DXL_HIWORD(goal_velocitys_trans[i])),
                                         DXL_HIBYTE(DXL_HIWORD(goal_velocitys_trans[i]))])
        self.write_data(id_list, self.ADDR_PRO_GOAL_VELOCITY, self.LEN_ADDR_PRO_GOAL_VELOCITY, param_goal_velocitys)
        return

    # '''
    # # 设置单个速度
    #     弃用，因为好像同步�?写会造成�??
    # '''
    #
    #
    # def set_velocity(id, goal_velocity):
    #     goal_velocity_trans = int(goal_velocity * 60 / (2 * math.pi * 0.229))
    #     param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(goal_velocity_trans)),
    #                            DXL_HIBYTE(DXL_LOWORD(goal_velocity_trans)),
    #                            DXL_LOBYTE(DXL_HIWORD(goal_velocity_trans)),
    #                            DXL_HIBYTE(DXL_HIWORD(goal_velocity_trans))]
    #     write_data([id], ADDR_PRO_GOAL_VELOCITY, LEN_ADDR_PRO_GOAL_VELOCITY, [param_goal_velocity])

    '''
    # 用速度模式，给定电机在各自当前位置以相同�?速度往前转
        输入：若干电机的编号（列�?��；�?速度（标量，弧度制）；转动时间（标量，单位是秒）
        过程�?
            切换模式到速度模式
            用while�?��当前距�?开始时间的时间间隔，判�?��没有到达指定�?��时间，到达就退�?
            把�?速度�?��成电机制，给各电机直接赋值速度
            切换模式回extend_position模式
        注意�?
            实际的轮式模式、rhex模式应�?采用速度模式，这样没有转的圈数的限制
            �?��作库函数，有while�?��
    
        如果isout为True，那么输入就指的�?��机连杆的数据
    '''

    def roll_vel_control(self, id_list, vel, durt):
        # read_data_dict
        keys = ['data_t', 'data_pos', 'data_vel', 'data_cur', 'data_vol']
        data_dict = dict(zip(keys, ([] for _ in keys)))

        t_start = time.time()
        print('==========')
        print('>>>>> roll_vel_control() Start!')

        self.switch_mode(id_list, 'velocity')

        # vel = int(vel * 60 / (2 * math.pi * 0.229))

        goal_velocitys = []
        for i in range(len(id_list)):
            goal_velocitys.append(vel)
        self.set_velocitys(id_list, goal_velocitys)

        t0 = time.time()
        t = 0
        read_data_t = []
        read_data_pos = []
        read_data_vel = []
        read_data_cur = []
        read_data_vol = []
        while t < durt:
            t = time.time() - t0
            # self.set_velocitys(id_list, goal_velocitys)

            # 读的都是电机的�?
            # read_t = time.time() - t0
            # read_pos = self.get_positions(id_list)
            # read_vel = self.get_velocitys(id_list)
            # read_vol = self.get_voltage(id_list)
            # read_cur = self.get_currents(id_list)
            # print('delta_t:\t', read_t)
            # # print('cur_pos:\t', read_pos)
            # print('cur_vel:\t', read_vel)
            # # print('cur_cur:\t', read_cur)
            # # print("cur_vol:\t", read_vol)
            # read_data_t.append(read_t)
            # read_data_pos.append(read_pos)
            # read_data_vel.append(read_vel)
            # read_data_cur.append(read_cur)
            # read_data_vol.append(read_vol)

        goal_velocitys = []
        for i in range(len(id_list)):
            goal_velocitys.append(0)
        self.set_velocitys(id_list, goal_velocitys)

        # datas to csv
        # df_pos = pd.DataFrame(read_data_pos, read_data_t, columns=range(len(id_list)))
        # df_vel = pd.DataFrame(read_data_vel, read_data_t, columns=range(len(id_list)))
        # df_cur = pd.DataFrame(read_data_cur, read_data_t, columns=range(len(id_list)))
        # df_vol = pd.DataFrame(read_data_vol, read_data_t, columns=range(len(id_list)))
        # df_pos.to_csv('./data/wheel/wheel_pos')
        # df_vel.to_csv('./data/wheel/wheel_vel')
        # df_cur.to_csv('./data/wheel/wheel_cur')
        # df_vol.to_csv('./data/wheel/wheel_vol')

        print('^^^^^ roll_vel_control() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        return read_data_t, read_data_pos, read_data_vel, read_data_cur

    def roll_vel_control_V2(self, id_list, vel, durt):

        self.switch_mode(self.id_list, 'velocity')
        goal_velocitys = []
        for _ in range(len(id_list)):
            goal_velocitys.append(vel)
        t0 = time.time()
        self.set_velocitys(id_list, goal_velocitys)
        while time.time() - t0 < durt:
            positions = self.get_positions(id_list)
            print('position:', positions)

        goal_velocitys = []
        for _ in range(len(id_list)):
            goal_velocitys.append(0)
        self.set_velocitys(id_list, goal_velocitys)
        self.switch_mode(id_list, 'extend_position')

    def roll_cycle(self, id_list, cyclenum):
        # vel = 2 * math.pi / 5 * gear_rate
        # t = cyclenum * gear_rate * 2 * math.pi / vel
        # roll_vel_control(id_list, vel, t)
        positions = self.get_positions(id_list)
        t0 = time.time()
        goal_positions = [i + cyclenum * 4096 * self.gear_rate for i in positions]
        self.set_allarrive_target_position(id_list, goal_positions)
        return

    '''
    设置速度超过最大速度，电机不会转
    '''

    def roll_rhex(self, id_list, vel_land, durt):

        t_start = time.time()
        print('==========')
        print('>>>>> roll_rhex() Start!')
        radio = 0.5  # 站立阶段的占空比
        dtheta = [] 

        t0 = time.time()

        # fai_land着地区域的范围
        fai_land = math.pi / 6

        fai_in_land = math.pi - 1 * fai_land / 2 
        fai_out_land = math.pi + 1 * fai_land / 2 

        t_inland = fai_land / vel_land

        vel_lift = (2 * math.pi - fai_land) / ((t_inland / radio) * (1 - radio))
        # flag = False
        # self.switch_mode(id_list, 'velocity')
        read_data_t = []
        read_data_pos = []
        read_data_vel = []
        read_data_cur = []
        read_data_vol = []
        # positions = [0] * len(id_list)

        while time.time() - t0 < durt:
            print('t:', time.time() - t0)
            positions = self.get_positions(id_list)
            # print(positions)
            leg_positions = []
            island = []
            for i in range(0, len(positions), 2):
                # 相位�?��这么直接�?��。这�?��加除以二，并且th2小于th1，中间值是朝上�?
                leg_positions.append((positions[i] + positions[i + 1]) / 2 + math.pi / math.pi * 2048)
            for i in range(len(leg_positions)):
                leg_positions[i] = leg_positions[i] % (4096)
                if  fai_in_land / math.pi * 2048 \
                        <= leg_positions[i] <= \
                        fai_out_land / math.pi * 2048:
                    island.append(True)
                    # flag = True
                    # print('in_time:', time.time())
                else:
                    island.append(False)
            # print([i / 2048 * math.pi for i in leg_positions])
            # for i in range(len(leg_positions)):
            #     positions[i] = [i / 2048 * math.pi for i in leg_positions]
            # for i in range(0, len(positions), 2):
            #     dtheta = positions[i] - positions[i+1]
            print(island)

            velocitys = []
            for i in range(len(leg_positions)):
                if island[i]:
                    velocitys.append(vel_land)
                    velocitys.append(vel_land)
                else:
                    velocitys.append(vel_lift)
                    velocitys.append(vel_lift)
            self.set_velocitys(id_list, velocitys)

            # 读的都是电机的�?
            read_t = time.time() - t0
            # read_pos = self.get_positions(id_list)
            # read_vel = self.get_velocitys(id_list)
            # read_cur = self.get_currents(id_list)
            # read_vol = self.get_voltage(id_list)
            # print('delta_t:\t', read_t)
            # print('cur_pos:\t', read_pos)
            # print('cur_vel:\t', read_vel)
            # print('cur_cur:\t', read_cur)
            # print('cur_vol:\t', read_vol)
            read_data_t.append(read_t)
            dtheta.append((leg_positions[0]-leg_positions[1] )/ 2048 * math.pi)
        
        plt.plot(read_data_t, dtheta, "g", marker='D', markersize=5)
        # "g" 表示红色，marksize用来设置'D'菱形的大小
        #绘制坐标轴标签
        plt.xlabel("时间")
        plt.ylabel("相位差")
        plt.legend(loc="lower right")
        plt.savefig("1.jpg")
        plt.show()
            # read_data_pos.append(read_pos)
            # read_data_vel.append(read_vel)
            # read_data_cur.append(read_cur)
            # read_data_vol.append(read_vol)

            # time.sleep(0.5)

        # datas to csv
        # df_pos = pd.DataFrame(read_data_pos, read_data_t, columns=range(len(id_list)))
        # df_vel = pd.DataFrame(read_data_vel, read_data_t, columns=range(len(id_list)))
        # df_cur = pd.DataFrame(read_data_cur, read_data_t, columns=range(len(id_list)))
        # df_vol = pd.DataFrame(read_data_vol, read_data_t, columns=range(len(id_list)))
        # df_pos.to_csv('./data/rhex/rhex_pos')
        # df_vel.to_csv('./data/rhex/rhex_vel')
        # df_cur.to_csv('./data/rhex/rhex_cur')
        # df_vol.to_csv('./data/rhex/rhex_vol')


        self.switch_mode(id_list, 'extend_position')

        print('^^^^^ roll_rhex() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        # return read_data_t, read_data_pos, read_data_vel, read_data_cur

    """
    由于压力会使腿型变，这里通过控制来缓解型�?
    """

    def roll_rhex_V2(self, id_list, vel_land, durt):
        t_start = time.time()
        print('==========')
        print('>>>>> roll_rhex() Start!')
        radio = 0.6  # 站立阶段的占空比

        # fai_land�?��地区域的范围
        fai_land = math.pi / 6

        fai_in_land = math.pi - 1 * fai_land / 2
        fai_out_land = math.pi + 1 * fai_land / 2

        t_inland = fai_land / vel_land

        vel_lift = (2 * math.pi - fai_land) / ((t_inland / radio) * (1 - radio))
        T = t_inland / radio
        # read_data_t = []
        # read_data_pos = []
        # read_data_vel = []
        # read_data_cur = []
        positions = [0] * len(id_list)
        t0 = time.time()
        while time.time() - t0 < durt:
            t = time.time() - t0
            t_cycle = t % T
            for i in range(len(positions)):
                if t_cycle < T / 2:
                    if i in [0, 2, 4]:

                        pass
                    else:
                        pass
                else:
                    if i in [0, 2, 4]:
                        pass
                    else:
                        pass

            # self.set_velocitys(id_list, velocitys)

            # 读的都是电机的�?
            # read_t = time.time() - t0
            # read_pos = self.get_positions(id_list)
            # read_vel = self.get_velocitys(id_list)
            read_cur = self.get_currents(id_list)
            # print('delta_t:\t', read_t)
            # print('cur_pos:\t', read_pos)
            # print('cur_vel:\t', read_vel)
            print('cur_cur:\t', read_cur)
            # read_data_t.append(read_t)
            # read_data_pos.append(read_pos)
            # read_data_vel.append(read_vel)
            # read_data_cur.append(read_cur)

        self.switch_mode(id_list, 'extend_position')

        print('^^^^^ roll_rhex() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        # return read_data_t, read_data_pos, read_data_vel, read_data_cur


    '''开环速度rhex模式'''
    def roll_rhex_V3(self, id_list, vel_land, durt):
        read_data_vol = []
        t_start = time.time()
        print('==========')
        print('>>>>> roll_rhex() Start!')
        radio = 0.5  # 站立阶段的占空比

        fai_land = math.pi / 6

        t_inland = fai_land / vel_land
        t_whole = t_inland/radio
        t_lift = t_whole*(1-radio)

        vel_lift = (2 * math.pi - fai_land) / ((t_inland / 0.5) * 0.5)
        velocitys = [0]*12
        '''
        original_pos = self.get_positions(id_list)
        goal_positions = [0] * len(id_list)
        for i in range(len(id_list)):
            goal_positions[i] = original_pos[i] + math.pi * 2048 * self.gear_rate 
        '''
        # 转半个周期
        for i in range(12):
            if (i // 2) % 2 == 0:
                velocitys[i] = vel_lift
            else:
                velocitys[i] = vel_land
        self.set_velocitys(id_list, velocitys)
        t1 = 0
        while t1 <  t_lift/2-0.15:
            t1 = time.time() - t_start

        # 后续时间
        t0 = time.time()
        t = 0
        while t < durt:
            t=time.time() - t0

            if (t%t_whole)<t_lift :
                mode = 0
            elif (t%t_whole)<t_whole:
                mode = 1

            velocitys = [0]*12
            for i in range(12):
                if mode==0:
                    if (i//2)%2 ==0:
                        velocitys[i]=vel_land
                    else:
                        velocitys[i]=vel_lift
                elif mode==1:
                    if (i//2)%2 ==0:
                        velocitys[i]=vel_lift
                    else:
                        velocitys[i]=vel_land
                #elif mode==2
                #    if (i/2)%2 ==0
                #    velocitys[i]=vel_land
                #    else
                #    velocitys[i]=vel_lift
            self.set_velocitys(id_list, velocitys)
            read_vel = self.get_velocitys(id_list)
            print('cur_vel:\t', read_vel)

        self.switch_mode(id_list, 'extend_position')

        print('^^^^^ roll_rhex() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)


    '''
    # 改变腿部的基础相位
        输入：若干电机的编号（列�?��；机器人模式名字（字符串：wheel、rhex�?
        过程：�?文档�?
    '''
    def roll_rhex_V4(self, id_list, vel_land, durt):

        t_start = time.time()
        print('==========')
        print('>>>>> roll_rhex Start!')

        goal_positions = self.get_positions(id_list)
        fai_land = math.pi / 3
        fai_in_land = math.pi  - 1 * fai_land / 2
        fai_out_land = math.pi  + 1 * fai_land / 2
        t_inland = fai_land / vel_land
        vel_lift = ((2 * math.pi - fai_land) / ((t_inland/0.5)*0.5))

        dposition_land = vel_land * self.dt * 4096 / (2 * math.pi)
        dposition_lift = vel_lift * self.dt * 4096 / (2 * math.pi)

        t0 = time.time()
        t = 0
        N = 0
        sample = 0
        read_data_t = []
        read_data_pos = []
        read_data_vel = []
        read_data_cur = []
        read_data_vol = []

        while t < durt:
            t = time.time() - t0
            if t > N * self.dt:

                #positions = self.get_positions(id_list)
                #print(positions)
                leg_positions = []

                for i in range(0, 12, 2):
                    # 相位可以这么直接转化。这个相加除以二，并且th2小于th1，中间值是朝上的
                    leg_positions.append((goal_positions[i] + goal_positions[i + 1]) / 2 + math.pi / math.pi * 2048)

                for i in range(len(leg_positions)):
                    leg_positions[i] = leg_positions[i] % (4096)
                    if (fai_in_land / math.pi * 2048) <= leg_positions[i] <= (fai_out_land / math.pi * 2048):
                        goal_positions[i * 2] = goal_positions[i * 2] + dposition_land
                        goal_positions[i * 2 + 1] = goal_positions[i * 2 + 1] + dposition_land
                        #print('in_time:', time.time())
                    else:
                        goal_positions[i * 2] = goal_positions[i * 2] + dposition_lift
                        goal_positions[i * 2 + 1] = goal_positions[i * 2 + 1] + dposition_lift

                self.set_positions(id_list, goal_positions)
                N = N + 1

                if t > sample * 0.15:
                    sample = sample + 1
                    # 读的都是电机的�?
                    # read_t = time.time() - t0
                    # #read_pos = self.get_positions(id_list)
                    # #read_vel = self.get_velocitys(id_list)
                    # read_cur = self.get_currents(id_list)
                    #read_vol = self.get_voltage(id_list)
                    #print('delta_t:\t', read_t)
                    #print('cur_pos:\t', read_pos)
                    #print('cur_pos:\t', read_vol)
                    #print('cur_vel:\t', read_vel)f
                    #print('cur_cur:\t', read_cur)
                    # read_data_t.append(read_t)
                    # #read_data_pos.append(read_pos)
                    # #read_data_vel.append(read_vel)
                    # read_data_cur.append(read_cur)
        # df_vel = pd.DataFrame(read_data_cur, read_data_t, columns=range(len(id_list)))
        # df_vel.to_csv('./data/rhex/rhex_cur')


        print('^^^^^ roll_pos_control() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        # return df_pos

    '''
    # 改变腿部的基础相位
        输入：若干电机的编号（列�?��；机器人模式名字（字符串：wheel、rhex�?
        过程：�?文档�?
    '''

    def mode_reset(self, id_list, robot_mode_name):
        # self.switch_mode(id_list, 'extend_position')

        dtheta = 0
        if robot_mode_name == self.wheel_mode:
            dtheta = (2 * math.pi / 3.0)
            positions = self.get_positions(id_list)
            positions_th1 = positions[::2]
            positions_th2 = positions[1::2]
            goal_positions = []
            for i in range(len(positions_th1)):
                goal_positions.append(int((positions_th1[i] + positions_th2[i]) / 2 + dtheta / math.pi * 2048 / 2))
                goal_positions.append(int((positions_th1[i] + positions_th2[i]) / 2 - dtheta / math.pi * 2048 / 2))

            print('当前位置：', positions)
            print('目标位置：', goal_positions)
            # self.set_allarrive_target_position(id_list, goal_positions)
            self.set_allarrive_target_position(id_list, goal_positions)
        elif robot_mode_name == self.rhex_mode:
            dtheta = (4 * math.pi / 3)
            positions = self.get_positions(id_list)
            positions_th1 = positions[::2]
            positions_th2 = positions[1::2]
            goal_positions = []
            for i in range(len(positions_th1)):
                goal_positions.append(int((positions_th1[i] + positions_th2[i]) / 2 + dtheta / math.pi * 2048 / 2))
                goal_positions.append(int((positions_th1[i] + positions_th2[i]) / 2 - dtheta / math.pi * 2048 / 2))
            print('当前位置：', positions)
            print('目标位置：', goal_positions)
            # self.set_allarrive_target_position(id_list, goal_positions)
            self.set_allarrive_target_position(id_list, goal_positions)

        elif robot_mode_name == self.leg_mode:
            positions = self.get_positions(id_list)
            # 判断腿的着地点
            xd = 0
            yd = self.step_H

            thetas = []
            th1, th2 = self.ikine.dpoint(xd, yd)
            for i in range(0, len(id_list), 2):
                thetas.append(th1 - 3 * math.pi / 2)
                thetas.append(th2 - 3 * math.pi / 2)
            goal_positions = []

            for i in range(len(thetas)):
                goal_positions.append(int(thetas[i] / math.pi * 2048))

            print('当前位置：', positions)
            print('目标位置：', goal_positions)
            # self.set_allarrive_target_position(id_list, goal_positions)
            self.set_allarrive_target_position(id_list, goal_positions)

    """
    腿模式行走
    """

    def walk_leg(self, id_list, durt):
        t_start = time.time()
        print('==========')
        print('>>>>> walk_leg() Start!')

        T = (self.walk_len / self.walk_vel) * 2

        t0 = time.time()
        read_data_t = []
        read_data_pos = []
        read_data_vel = []
        read_data_cur = []
        read_data_vol = []

        while time.time() - t0 < durt:
            t_incycle = (time.time() - t0) % T

            # 右三角先往后迈，也就是先直�?
            xd = []
            yd = []
            for i in range(len(id_list)):
                if t_incycle <= (T / 2):
                    if i in self.id_list_right:  # 右腿后迈
                        xd.append((-1) * (2 * self.walk_len / 4) + t_incycle * self.walk_vel)
                        yd.append(self.step_H)
                    else:  # 左腿前迈
                        xd.append((2 * self.walk_len / 4) - t_incycle * self.walk_vel)
                        # yd = [steph * math.sin(2 * math.pi / (stepL_len * 2) * i) + stepH for i in xd]
                        yd.append(self.step_h * math.sin(
                            (xd[i] + 1 * self.walk_len / 5) / self.walk_len * math.pi) + self.step_H)
                else:
                    if i in self.id_list_right:  # 右腿前迈
                        xd.append((2 * self.walk_len / 4) - (t_incycle - T / 2) * self.walk_vel)
                        # yd = [steph * math.sin(2 * math.pi / (stepL_len * 2) * i) + stepH for i in xd]
                        yd.append(self.step_h * math.sin(
                            (xd[i] + 2 * self.walk_len / 4) / self.walk_len * math.pi) + self.step_H)
                    else:  # 左腿后迈
                        xd.append((-1) * (2 * self.walk_len / 4) + (t_incycle - T / 2) * self.walk_vel)
                        # yd = [stepH] * len(id_list)
                        yd.append(self.step_H)

            # print('xd', xd)
            # print('yd', yd)

            # 这个thetas�?��的腿的th1和th2的�?�?
            # 并且th2�??数，�??一�?��期里的；且th1比th2大，�?���?���?��期里的，也是�??�?
            thetas = []
            for i in range(0, len(id_list), 2):
                th1, th2 = self.ikine.dpoint(xd[i], yd[i])
                thetas.append(th1 - 3 * math.pi / 2)
                thetas.append(th2 - 3 * math.pi / 2)

            goal_positions = [i / math.pi * 2048 for i in thetas]
            self.set_positions(id_list, goal_positions)

            # 读的都是电机的值
            
            
            read_t = time.time() - t0
            read_pos = self.get_positions(id_list)
            # read_vel = self.get_velocitys(id_list)
            # read_cur = self.get_currents(id_list)
            # read_vol = self.get_voltage(id_list)
            '''
            # print('delta_t:\t', read_t)
            # print('cur_pos:\t', read_pos)
            # print('cur_vel:\t', read_vel)
            # print('cur_cur:\t', read_cur)
            read_data_t.append(read_t)
            read_data_pos.append(read_pos)
            read_data_vel.append(read_vel)
            read_data_cur.append(read_cur)
            read_data_vol.append(read_vol)
            '''
            

        # datas to csv
        df_pos = pd.DataFrame(read_data_pos, read_data_t, columns=range(len(id_list)))
        # df_vel = pd.DataFrame(read_data_vel, read_data_t, columns=range(len(id_list)))
        # df_cur = pd.DataFrame(read_data_cur, read_data_t, columns=range(len(id_list)))
        # df_vol = pd.DataFrame(read_data_vol, read_data_t, columns=range(len(id_list)))
        df_pos.to_csv('./data/leg/leg_pos')
        # df_vel.to_csv('./data/leg/leg_vel')
        # df_cur.to_csv('./data/leg/leg_cur')
        # df_vol.to_csv('./data/leg/leg_vol')
        

        print('^^^^^ walk_leg() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        # return read_data_t, read_data_pos, read_data_vel, read_data_cur

    """
    复位偏移量
    """

    def reset_offset(self):
        # 关闭力矩
        self.switch_torque(self.id_list, 'disable')
        reset_value = []
        for _ in range(len(self.id_list)):
            reset_value.append([DXL_LOBYTE(DXL_LOWORD(0)),
                                DXL_HIBYTE(DXL_LOWORD(0)),
                                DXL_LOBYTE(DXL_HIWORD(0)),
                                DXL_HIBYTE(DXL_HIWORD(0))])

        self.write_data(self.id_list, self.ADDR_PRO_HOMING_OFFSET, self.LEN_PRO_HOMING_OFFSET, reset_value)

        self.switch_torque(self.id_list, 'disable')

    """
    在扩展模式下
    设置电机的偏移量
    """

    def offset_position_EXT(self):

        self.reset_offset()

        # 关闭力矩
        self.switch_torque(self.id_list, 'disable')
        positions = self.get_positions(self.id_list)
        offset_values = []
        for i in range(0, len(self.id_list), 2):
            if i + 1 < 6:
                # 外侧腿的偏移量
                print('%d号电机的位置为%03d' % (i, positions[i]))
                offset_vaule = (-1) * positions[i] + int((math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

                # 内侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i + 1, positions[i + 1]))
                offset_vaule = (-1) * positions[i + 1] + int((-math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i + 1], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])


            else:
                # 外侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i, positions[i]))
                offset_vaule = positions[i] - int((math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

                # 内侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i + 1, positions[i + 1]))
                offset_vaule = positions[i + 1] - int((-math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i + 1], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

        self.write_data(self.id_list, self.ADDR_PRO_HOMING_OFFSET, self.LEN_PRO_HOMING_OFFSET, offset_values)

    """
    这是为了腿模式下方便使用才写的
    """

    def offset_position(self):

        self.reset_offset()

        # 关闭力矩
        self.switch_torque(self.id_list, 'disable')
        positions = self.get_positions(self.id_list)
        offset_values = []
        for i in range(0, len(self.id_list), 2):
            if i + 1 < 6:
                # 外侧腿的偏移量
                print('%d号电机的位置为%03d' % (i, positions[i]))
                offset_vaule = (-1) * positions[i] + int((math.pi + math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

                # 内侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i + 1, positions[i + 1]))
                offset_vaule = (-1) * positions[i + 1] + int((math.pi - math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i + 1], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])


            else:
                # 外侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i, positions[i]))
                offset_vaule = positions[i] - int((math.pi + math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

                # 内侧腿的偏移量
                print('%d号电机的位置�?%03d' % (i + 1, positions[i + 1]))
                offset_vaule = positions[i + 1] - int((math.pi - math.pi / 3) / math.pi * 2048)
                print('%d号电机的偏移量为%03d' % (self.id_list[i + 1], offset_vaule))
                offset_values.append([DXL_LOBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_LOWORD(offset_vaule)),
                                      DXL_LOBYTE(DXL_HIWORD(offset_vaule)),
                                      DXL_HIBYTE(DXL_HIWORD(offset_vaule))])

        self.write_data(self.id_list, self.ADDR_PRO_HOMING_OFFSET, self.LEN_PRO_HOMING_OFFSET, offset_values)

    """
    判断电机的位�?��否合理，即是否能够直接直接�?�?
    输入：电机�?
    输出：True（合理）�? False（不合理�?
    """

    def is_valid(self, id_list):
        positions = self.get_positions(id_list)
        for i in range(0, len(positions), 2):
            if positions[i] > positions[i + 1] and \
                    positions[i] - positions[i + 1] < 4096:
                return True
            else:
                print('[ID:%d]invalid position!!!!' % i)
                print(positions[i], positions[i + 1])
                return False

    def reset_just(self, id_list):

        # 判断当前电机的位置是否合理（达到复位要求），若不合理则直接退出
        if self.is_valid(id_list):
            # pass
            print('可以复位...')
        else:
            quit()

        positions = self.get_positions(id_list)
        # 如果是位置模式的话不用考虑周期问题，也就是base可以不用考虑
        count = [0] * len(id_list)
        base = count.copy()
        for i in range(0, len(positions), 2):
            # while positions[i] >= 4096:
            #     positions[i] -= 4096
            #     count[i] += 1
            count[i] = int(positions[i] / (4096)) - 1
            base[i] = count[i] * (4096)
            count[i + 1] = count[i]
            base[i + 1] = base[i]

        init_pos = [int((math.pi / 3) / math.pi * 2048), int((- math.pi / 3) / math.pi * 2048)] * 6

        print('当前位置:', positions)
        print('目标位置:', init_pos)

        self.set_allarrive_target_position_V2(id_list, init_pos, 3)

        # self.switch_mode(id_list, mode_name='position')
        # self.switch_mode(id_list, mode_name='extend_position')

        # self.switch_torque(self.id_list, 'disable')

        return

    # 从初始化相位到腿模式（�?求先执�?reset_just�?
    # 初始化是rest_just，变�?6�?��同相位的�?��模式
    # 然后要动或者�?换模式先�?��行这�?��先站起来
    def inittoleg(self):
        # 首先所有腿先变换成腿模式
        self.mode_reset(self.id_list, self.leg_mode)
        # if input('按下任意键始（e退出）：') != 'e':
        #     pass
        # 然后左腿后迈，右腿前迈
        t0 = time.time()
        durt = 3  # 变换需要的时间
        xd = [0] * len(self.id_list)
        yd = [0] * len(self.id_list)
        while time.time() - t0 < durt:
            for i in self.id_list:
                if i in self.id_list_left:  # 左腿水平后迈
                    xd[i] = (time.time() - t0) / durt * (2 * self.walk_len / 4)
                    yd[i] = self.step_H
                else:  # 右腿抬腿前迈
                    xd[i] = (-1) * (time.time() - t0) / durt * (2 * self.walk_len / 4)
                    yd[i] = (-1) * self.step_h * math.sin((xd[i] / (2 * self.walk_len / 4)) * math.pi) + self.step_H

            thetas = []
            for i in range(0, len(self.id_list), 2):
                th1, th2 = self.ikine.dpoint(xd[i], yd[i])
                thetas.append(th1 - 3 * math.pi / 2)
                thetas.append(th2 - 3 * math.pi / 2)
            goal_positions = [i / math.pi * 2048 for i in thetas]
            self.set_positions(self.id_list, goal_positions)
        return

    # 在刚刚的基础上，还要考虑腿的初�?高度，腿的三三�?态相位！
    def legtoleginit(self):
        self.mode_reset(self.id_list_left, self.leg_mode, 'point2')
        return

    # 由腿模式切换到轮模式
    def legtowheel(self, id_list):
        pass

    # 由腿模式切换到RHEX模式
    def legtorhex(self, id_list):
        pass

    # 从初始化相位到轮模式（�?求先执�?reset_just�?
    # 要先到腿模式站起�?
    # 然后左侧腿收起来，收到轮模式
    # 然后左侧腿往前转半个周期
    # 然后右侧腿收起来，收到腿模式
    def inittowheel(self):
        # 所有腿变换到rhex模式，机器人站起来
        self.mode_reset(self.id_list, self.rhex_mode)
        # 关力矩，不然会�?�?
        # self.switch_torque(self.id_list_right, 'disable')
        # 左三腿切换回轮模式
        self.mode_reset(self.id_list_left, self.wheel_mode)
        # 左三腿的电机切换到速度模式
        self.switch_mode(self.id_list_left, 'velocity')
        # 左三腿转动半个周期
        self.roll_vel_control(self.id_list_left, math.pi / 2, 2)

        # 右三腿切换回轮模式
        self.mode_reset(self.id_list_right, self.wheel_mode)

        # 右三腿的电机切换到速度模式
        self.switch_mode(self.id_list_right, 'velocity')
        return

    #  由轮模式切换到RHEX模式
    def wheeltorhex(self, id_lsit):
        pass

    # 由轮模式切换到腿模式
    def wheeltoleg(self, id_lsit):
        pass

    # 从初始化相位到rhex模式（�?求先执�?reset_just�?
    # 要先到腿模式站起�?
    # 然后左侧腿收起来，收到轮模式
    # 然后左侧腿往前转半个周期
    # 然后左侧腿变成腿模式
    def inittorhex(self):
        self.mode_reset(self.id_list_right, self.rhex_mode)

        # 关力矩，不然会塌掉
        # self.switch_torque(self.id_list, 'disable')
        print("================================")
        #self.mode_reset(self.id_list_right, self.wheel_mode)
        self.roll_cycle(self.id_list_left, 0.5)
        self.mode_reset(self.id_list_left, self.rhex_mode)
        
        #if input('按下任意键开始（e退出）') != 'e':
        #    self.roll_cycle(self.id_list, -1/24)
        
        # 再开力矩
        self.switch_mode(self.id_list, 'velocity')
        self.switch_torque(self.id_list_left, 'enable')
        return

    def init_to_rhexinit(self):
        pass

    # 由RHEX模式切换到轮模式
    def rhextowheel(self, id_list):
        pass

    # 由RHEX模式切换到腿模式
    def rhextoleg(self, id_list):
        pass

    def roll_rhex_feedback(self, id_list, vel_land, durt):
        t0 = time.time()

        # 角度设定的是外部腿的角度，所以是外面的，要换到电机的相位
        fai_land = math.pi / 6
        fai_in_land = math.pi - 11 * fai_land / 20
        fai_out_land = math.pi + 9 * fai_land / 20

        t_inland = fai_land / vel_land
        vel_lift = (2 * math.pi - fai_land) / t_inland

        # self.switch_mode(id_list, 'velocity')

        while time.time() - t0 < durt:
            positions = self.get_positions(id_list)
            print('t:', time.time() - t0, positions)
            leg_positions = []
            island = []
            for i in range(0, len(positions), 2):
                # 相位�?��这么直接�?��。捏�?��，这�?��加除以二，并且th2小于th1，中间值是朝上�?
                leg_positions.append((positions[i] + positions[i + 1]) / 2 + math.pi / math.pi * 2048 * self.gear_rate)
            for i in range(len(leg_positions)):
                leg_positions[i] = leg_positions[i] % (4096 * self.gear_rate)
                if fai_in_land / math.pi * 2048 \
                        <= leg_positions[i] <= \
                        fai_out_land / math.pi * 2048:
                    island.append(True)
                else:
                    island.append(False)

            # 反�?�?��几�?
            # 计算剩余多少时间，然后�?算还剩�?少路程�?�?
            # 计算多少�?��要跑要分为着地的和没着地的，其�?��着地的因为周期的关系还要再讨�?
            t_inhalfcyc = (time.time() - t0) % t_inland
            t_halfremain = t_inland - t_inhalfcyc
            deltaPos = []
            for i in range(len(leg_positions)):
                if island[i] == True:
                    deltaPos.append(fai_out_land / math.pi * 2048 - leg_positions[i])
                else:
                    if leg_positions[i] >= fai_out_land / math.pi * 2048:
                        deltaPos.append(4096 * self.gear_rate - leg_positions[i] + fai_in_land)
                    else:
                        deltaPos.append(fai_in_land - leg_positions[i])

            velocitys = [i / 2048 * math.pi / t_halfremain for i in deltaPos]
            velocity = []
            for i in range(len(velocitys)):
                velocity.append(velocitys[i])
                velocity.append(velocitys[i])

            print(len(velocity))
            # velocitys = []
            # for i in range(len(leg_positions)):
            #     if island[i]:
            #         velocitys.append(vel_land)
            #         velocitys.append(vel_land)
            #     else:
            #         velocitys.append(vel_lift)
            #         velocitys.append(vel_lift)
            self.set_velocitys(id_list, velocity)

        # self.switch_mode(id_list, 'extend_position')
        return

    def legpos_control(self, x, y, leg_id):
        thetas = [0] * 2
        id_list = [2 * leg_id, 2 * leg_id + 1]
        th1, th2 = self.ikine.dpoint(x, y)
        thetas[0] = th1 - 3 * math.pi / 2
        thetas[1] = th2 - 3 * math.pi / 2

        goal_positions = [i / math.pi * 2048 for i in thetas]

        self.set_positions(id_list, goal_positions)

    def set_provelocitys(self, id_list, pro_vel):
        profile_vel = []
        for i in range(len(id_list)):
            profile_vel.append([DXL_LOBYTE(DXL_LOWORD(pro_vel)),
                                DXL_HIBYTE(DXL_LOWORD(pro_vel)),
                                DXL_LOBYTE(DXL_HIWORD(pro_vel)),
                                DXL_HIBYTE(DXL_HIWORD(pro_vel))])
        self.write_data(id_list, self.ADDR_PRO_PROFILE_VELOCITY, self.LEN_ADDR_PRO_PROFILE_VELOCITY, profile_vel)

    def initandclimb(self, ladder_width):
        pro_vel = 800
        profile_vel = []
        for i in range(len(self.id_list)):
            profile_vel.append([DXL_LOBYTE(DXL_LOWORD(pro_vel)),
                                DXL_HIBYTE(DXL_LOWORD(pro_vel)),
                                DXL_LOBYTE(DXL_HIWORD(pro_vel)),
                                DXL_HIBYTE(DXL_HIWORD(pro_vel))])
        # print("profile_vel%d" %(profile_vel))

        self.write_data(self.id_list, self.ADDR_PRO_PROFILE_VELOCITY, self.LEN_ADDR_PRO_PROFILE_VELOCITY, profile_vel)

        xd = [0 for _ in range(6)]
        yd = [0 for _ in range(6)]
        # read_data_dict
        keys = ['data_t', 'data_pos', 'data_vel', 'data_cur', 'data_vol']
        data_dict = dict(zip(keys, ([] for _ in keys)))

        for i in range(0, 6):
            xd[i] = 0 * 2 * self.r
            yd[i] = -2 * self.r
            self.legpos_control(xd[i], yd[i], i)
        if input('按下任意键开始（e退出）') != 'e':
            pass
        #####################################################################################
        # 爬木梯子初始位置
        init_pos = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r - 0.003, -0.65 * 2 * self.r], [-0.1 * 2 * self.r - 0.01, -0.65 * 2 * self.r],
                    [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r - 0.003, -0.65 * 2 * self.r], [-0.1 * 2 * self.r - 0.01, -0.65 * 2 * self.r]]

        # 爬铁梯子初始位置
        # init_pos = [[-0.04 * 2 * self.r, -0.63 * 2 * self.r], [0.155 * 2 * self.r, -0.6 * 2 * self.r], [-0.26 * 2 * self.r, -0.55 * 2 * self.r],
        #             [-0.04 * 2 * self.r, -0.63 * 2 * self.r], [0.155 * 2 * self.r, -0.6 * 2 * self.r], [-0.26 * 2 * self.r, -0.55 * 2 * self.r]]

        for i in range(6):
            self.legpos_control(init_pos[i][0], init_pos[i][1], i)

        print('==========')
        print('>>>>> climb_ladder() Start!')

        # t_start = time.time()
        # self.record_save(data_dict, self.id_list, t_start, durt=5)
        if input('按下任意键开始（e退出）') != 'e':
            pass
        t_start = time.time()

        for i in range(2):
            self.climb_ladder_cycle_V1(ladder_width, t_start, data_dict)
        #
        #     # 回到初始位置
        #
            for j in range(6):
                self.legpos_control(init_pos[j][0], init_pos[j][1], j)

            # self.record_save(data_dict, self.id_list, t_start, durt=5)
        #
            # if input('按下任意键开始（e退出）') != 'e':
            #     pass

        # datas to csv
        # df_pos = pd.DataFrame(data_dict['data_pos'], data_dict['data_t'], columns=range(12))
        # df_vel = pd.DataFrame(data_dict['data_vel'], data_dict['data_t'], columns=range(12))
        df_cur = pd.DataFrame(data_dict['data_cur'], data_dict['data_t'], columns=range(12))
        # df_vol = pd.DataFrame(data_dict['data_vol'], data_dict['data_t'], columns=range(12))
        # df_pos.to_csv('./data/climb_pos')
        # df_vel.to_csv('./data/climb_vel')
        df_cur.to_csv('./data/climb_cur')
        # df_vol.to_csv('./data/climb_vol')

    # here we suppose that the keys of data:dict are checked and static
    # award but never mind, just for shortening the lines
    def record_save(self, data: dict, id_list: list, t_start, durt):
        sample = 0
        t0 = time.time()
        while time.time() - t0 < durt:
            if time.time() - t0 > sample * 0.10:
                sample = sample + 1
                read_t = time.time() - t_start
                # read_pos = self.get_positions(id_list)
                # read_vel = self.get_velocitys(id_list)
                read_cur = self.get_currents(id_list)
                # read_vol = self.get_voltage(id_list)
                data['data_t'].append(read_t)
                # data['data_pos'].append(read_pos)
                # data['data_vel'].append(read_vel)
                data['data_cur'].append(read_cur)
                # data['data_vol'].append(read_vol)
        # return data

    # 爬木梯子的函数
    def climb_ladder_cycle_V1(self, ladder_width, t_start, data_dict):
        xd = [0 for _ in range(6)]
        yd = [0 for _ in range(6)]
        
        # # 中腿抓紧
        # for i in range(1, 6, 3):
        #     xd[i] = -0.1 * 2 * self.r
        #     yd[i] = -0.6 * 2 * self.r
        #     self.legpos_control(xd[i], yd[i], i)
        # # 前腿后移
        # for i in range(0, 6, 3):
        #     xd[i] = -0.1 * 2 * self.r - 0.005
        #     yd[i] = -0.65 * 2 * self.r
        #     self.legpos_control(xd[i], yd[i], i)
        # time.sleep(1)

        step = []
        
        # 后腿运动
        #####################################################################
        # 后腿前进一段距离
        step_0 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.02, -0.65 * 2 * self.r],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.02, -0.65 * 2 * self.r]]
        # 后腿斜向上运动
        step_1 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.03],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.03]]
        # 后腿斜向上运动
        step_2 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.05],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.05]]
        # 后腿斜向下运动
        step_3 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.17, -0.65 * 2 * self.r + 0.06],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.17, -0.65 * 2 * self.r + 0.06]]
        # 后退垂直向下运动
        step_4 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r + 0.02],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r + 0.02]]
        # 后腿向后运动
        step_5 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)
        
        tdurt = [0.5, 0.7, 0.8, 0.8, 0.7, 0.7]
        
        for i in range(6):
            print("step%d", i)
            for j in range(2, 6, 3):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
            if input('按下任意键开始（e退出）') != 'e':
                pass

        step = []
        # 中腿运动
        ####################################################################################
        # 中腿前进一段距离
        step_0 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.02, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.02, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02]]
        # 中腿斜向上运动
        step_1 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02]]
        # 中腿斜向上运动,同时其他腿后退
        step_2 = [[-0.1 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.05], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.05], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.02]]
        # 中腿斜向下运动
        step_3 = [[-0.1 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.18, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.03],
                  [-0.1 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.18, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.03]]
        # 中腿垂直向下运动
        step_4 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.13, -0.65 * 2 * self.r - 0.01], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r - 0.03],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.13, -0.65 * 2 * self.r - 0.01], [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r - 0.03]]
        # 中腿向后运动
        step_5 = [[-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.03],
                  [-0.1 * 2 * self.r, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.03]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)
        
        tdurt = [0.5, 0.6, 0.8, 0.8, 0.7, 0.9]
        
        for i in range(6):
            print("step%d", i)
            for j in range(1, 6, 3):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
            if input('按下任意键开始（e退出）') != 'e':
                pass

        step = []
        # 前腿运动
        ##############################################################################
        # 前腿向前运动
        step_0 = [[-0.1 * 2 * self.r + 0.025, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.025, -0.65 * 2 * self.r], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.02]]
        # 前腿斜向上运动, 同时后腿和中腿向下
        step_1 = [[-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.05], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.11, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.07, -0.65 * 2 * self.r + 0.05], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.11, -0.65 * 2 * self.r - 0.02]]
        # 前腿斜向上运动
        step_2 = [[-0.1 * 2 * self.r + 0.16, -0.65 * 2 * self.r + 0.06], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.11, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.16, -0.65 * 2 * self.r + 0.06], [-0.1 * 2 * self.r + 0.1, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.11, -0.65 * 2 * self.r - 0.02]]
        # 前腿斜向下运动
        step_3 = [[-0.1 * 2 * self.r + 0.19, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.19, -0.65 * 2 * self.r + 0.03], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02]]
        # 前腿垂直向下运动
        step_4 = [[-0.1 * 2 * self.r + 0.17, -0.65 * 2 * self.r - 0.01], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.17, -0.65 * 2 * self.r - 0.01], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02]]
        # 前腿向后运动
        step_5 = [[-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02],
                  [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02], [-0.1 * 2 * self.r + 0.08, -0.65 * 2 * self.r - 0.02]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)
        
        tdurt = [0.6, 0.8, 0.9, 0.9, 0.8, 0.9]
        
        for i in range(6):
            print("step%d", i)
            for j in range(0, 6, 1):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
            if input('按下任意键开始（e退出）') != 'e':
                pass

    # 爬铁梯子的函数
    def climb_ladder_cycle_V2(self, ladder_width, t_start, data_dict):
        xd = [0 for _ in range(6)]
        yd = [0 for _ in range(6)]

        # # 中腿抓紧
        # for i in range(1, 6, 3):
        #     xd[i] = -0.1 * 2 * self.r
        #     yd[i] = -0.6 * 2 * self.r
        #     self.legpos_control(xd[i], yd[i], i)
        # # 前腿后移
        # for i in range(0, 6, 3):
        #     xd[i] = -0.1 * 2 * self.r - 0.005
        #     yd[i] = -0.65 * 2 * self.r
        #     self.legpos_control(xd[i], yd[i], i)
        # time.sleep(1)

        step = []
        # 后腿运动
        #####################################################################
        # 后腿前进一段距离
        step_0 = [[-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.02, -0.55 * 2 * self.r],
                  [-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.02, -0.55 * 2 * self.r]]
        # 后腿斜向上运动,其他两腿后退
        step_1 = [[0.02 * 2 * self.r, -0.6 * 2 * self.r], [0.19 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.07, -0.55 * 2 * self.r + 0.02],
                  [0.02 * 2 * self.r, -0.6 * 2 * self.r], [0.19 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.07, -0.55 * 2 * self.r + 0.02]]
        # 后腿斜向上运动
        step_2 = [[0.1 * 2 * self.r, -0.6 * 2 * self.r], [0.19 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.15, -0.55 * 2 * self.r + 0.03],
                  [0.1 * 2 * self.r, -0.6 * 2 * self.r], [0.19 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.15, -0.55 * 2 * self.r + 0.03]]
        # 后腿斜向下运动
        step_3 = [[0.1 * 2 * self.r, -0.57 * 2 * self.r], [0.19 * 2 * self.r, -0.65 * 2 * self.r], [-0.3 * 2 * self.r + 0.20, -0.55 * 2 * self.r + 0.01],
                  [0.1 * 2 * self.r, -0.57 * 2 * self.r], [0.19 * 2 * self.r, -0.65 * 2 * self.r], [-0.3 * 2 * self.r + 0.20, -0.55 * 2 * self.r + 0.01]]
        # 后退垂直向下运动
        step_4 = [[0.1 * 2 * self.r, -0.55 * 2 * self.r], [0.19 * 2 * self.r, -0.68 * 2 * self.r], [-0.3 * 2 * self.r + 0.20, -0.55 * 2 * self.r + -0.03],
                  [0.1 * 2 * self.r, -0.55 * 2 * self.r], [0.19 * 2 * self.r, -0.68 * 2 * self.r], [-0.3 * 2 * self.r + 0.20, -0.55 * 2 * self.r + -0.03]]
        # 后腿向后运动
        step_5 = [[-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.13, -0.55 * 2 * self.r],
                  [-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.13, -0.55 * 2 * self.r]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)

        tdurt = [0.5, 0.7, 0.8, 0.8, 0.7, 0.9]

        for i in range(6):
            print("step%d", i)
            for j in range(0, 6):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
            # if input('按下任意键开始（e退出）') != 'e':
            #     pass
            # if i == 0 and i == 5:
            #     self.record_save(data_dict, self.id_list, t_start, durt=5)

        step = []
        # 中腿运动
        ####################################################################################
        # 中腿前进一段距离
        step_0 = [[-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.02, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.12, -0.55 * 2 * self.r],
                  [-0.05 * 2 * self.r, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.02, -0.6 * 2 * self.r], [-0.3 * 2 * self.r + 0.12, -0.55 * 2 * self.r]]
        # 中腿斜向上运动. 同时其他两腿向上运动
        step_1 = [[-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.07, -0.6 * 2 * self.r + 0.02], [-0.3 * 2 * self.r + 0.1, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.07, -0.6 * 2 * self.r + 0.02], [-0.3 * 2 * self.r + 0.1, -0.6 * 2 * self.r]]
        # 中腿斜向上运动
        step_2 = [[-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.14, -0.6 * 2 * self.r + 0.05], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r],
                  [-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.14, -0.6 * 2 * self.r + 0.05], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r]]
        # 中腿斜向下运动
        step_3 = [[-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.17, -0.6 * 2 * self.r + 0.03], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r],
                  [-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.17, -0.6 * 2 * self.r + 0.03], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r]]
        # 中腿垂直向下运动
        step_4 = [[-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.14, -0.6 * 2 * self.r - 0.02], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r],
                  [-0.05 * 2 * self.r - 0.02, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.14, -0.6 * 2 * self.r - 0.02], [-0.3 * 2 * self.r + 0.1, -0.62 * 2 * self.r]]
        # 中腿向后运动
        step_5 = [[-0.05 * 2 * self.r - 0.025, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.1, -0.6 * 2 * self.r - 0.027], [-0.3 * 2 * self.r + 0.095, -0.62 * 2 * self.r],
                  [-0.05 * 2 * self.r - 0.025, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.1, -0.6 * 2 * self.r - 0.027], [-0.3 * 2 * self.r + 0.095, -0.62 * 2 * self.r]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)

        tdurt = [0.5, 0.7, 0.8, 0.8, 0.7, 0.9]

        for i in range(6):
            print("step%d", i)
            for j in range(0, 6):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
            # if input('按下任意键开始（e退出）') != 'e':
            #     pass

        step = []

        # 前腿运动
        ##############################################################################
        # 前腿向前运动
        step_0 = [[-0.05 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.07, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.09, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.01, -0.65 * 2 * self.r], [0.17 * 2 * self.r + 0.07, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.09, -0.6 * 2 * self.r]]
        # 前腿斜向上运动, 同时后腿和中腿向下
        step_1 = [[-0.05 * 2 * self.r + 0.05, -0.65 * 2 * self.r + 0.015], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.04], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.05, -0.65 * 2 * self.r + 0.015], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.04], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r]]
        # 前腿斜向上运动
        step_2 = [[-0.05 * 2 * self.r + 0.1, -0.65 * 2 * self.r + 0.035], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.1, -0.65 * 2 * self.r + 0.035], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r]]
        # 前腿斜向下运动
        step_3 = [[-0.05 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.02], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.15, -0.65 * 2 * self.r + 0.02], [0.17 * 2 * self.r + 0.06, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r]]
        # 前腿垂直向下运动
        step_4 = [[-0.05 * 2 * self.r + 0.12, -0.65 * 2 * self.r + 0.015], [0.17 * 2 * self.r + 0.062, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.12, -0.65 * 2 * self.r + 0.015], [0.17 * 2 * self.r + 0.062, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.08, -0.6 * 2 * self.r]]
        # 前腿向后运动
        step_5 = [[-0.05 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.01], [0.17 * 2 * self.r + 0.063, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.075, -0.6 * 2 * self.r],
                  [-0.05 * 2 * self.r + 0.09, -0.65 * 2 * self.r - 0.01], [0.17 * 2 * self.r + 0.063, -0.6 * 2 * self.r - 0.045], [-0.3 * 2 * self.r + 0.075, -0.6 * 2 * self.r]]

        step.append(step_0)
        step.append(step_1)
        step.append(step_2)
        step.append(step_3)
        step.append(step_4)
        step.append(step_5)

        tdurt = [0.6, 0.8, 0.9, 0.9, 0.8, 0.6]

        for i in range(6):
            print("step%d", i)
            for j in range(0, 6, 1):
                self.legpos_control(step[i][j][0], step[i][j][1], j)
            self.record_save(data_dict, self.id_list, t_start, durt=tdurt[i])
        #     if input('按下任意键开始（e退出）') != 'e':
        #         pass
        #     if i == 1 and i == 5:
        #         self.record_save(data_dict, self.id_list, t_start, durt=1)

    def showoff(self):
        self.reset_just(self.id_list)
        self.inittoleg(self.id_list)
        self.walk_leg(self.id_list, durt=20)

        self.reset_just(self.id_list)
        self.inittorhex(self.id_list)
        # print('positions:', self.get_positions(self.id_list))
        self.roll_rhex(self.id_list, vel_land=1 * math.pi / 20.0, durt=20.0)

        self.reset_just(self.id_list)
        self.inittowheel(self.id_list)
        self.roll_cycle(self.id_list, 1)

    def finished(self):
        # self.reset_just(self.id_list)
        self.switch_torque(self.id_list, 'disable')
        self.portHandler.closePort()

    def climb_stair_init_V1(self, stair_height, stair_width):
        stair_angle = math.tan(stair_height / stair_width)
        # leg_angle_hou = math.pi + stair_angle + math.acos(leg_height_hou / (2*self.r))
        # leg_angle_zhong = math.pi + stair_angle - \
        #     math.acos((0.215 * math.sin(stair_angle) + leg_height_hou - stair_height - self.r) / self.r)
        # leg_angle_qian = math.pi + stair_angle - \
        #     math.acos((0.43 * math.sin(stair_angle) + leg_height_hou - 2*stair_height - self.r) / self.r)

        leg_angle_hou = math.pi + 1.2 * math.pi / 4
        leg_angle_zhong = math.pi - math.pi / 6
        leg_angle_qian = math.pi - 1 * math.pi / 3

        self.mode_reset(self.id_list, self.rhex_mode)

        # 后腿转动
        self.roll_angle(self.id_list_hou, leg_angle_hou - math.pi)
        # 中腿转动
        self.roll_angle(self.id_list_zhong, leg_angle_zhong - math.pi)
        # 前腿转动
        self.roll_angle(self.id_list_qian, leg_angle_qian - math.pi)

        self.switch_mode(self.id_list, 'velocity')

    # 另一种初始状态
    # 这里考虑楼梯高度为12cm，宽度为30cm
    def climb_stair_init_V2(self, stair_height, stair_width):
        leg_angle_hou = math.pi
        leg_angle_qian = math.pi

        self.mode_reset(self.id_list, self.rhex_mode)

        # 后腿转动
        self.roll_angle(self.id_list_hou, leg_angle_hou)
        # 前腿转动
        self.roll_angle(self.id_list_qian, leg_angle_qian)

        # 中腿转动

    def climb_stair_walk_V1(self, height, width, count):
        stair_angle = math.tan(height / width)

        leg_angle_hou_stage0 = math.pi + 1.2 * math.pi / 4
        leg_angle_zhong_stage0 = math.pi - math.pi / 6
        leg_angle_qian_stage0 = math.pi - math.pi / 3

        ################   阶段一  ###########################
        leg_vel_hou_stage1 = [math.pi / 3] * 4
        leg_angle_hou_stage1 = 2 * math.pi * count + 9 * math.pi / 24
        self.switch_mode(self.id_list, 'velocity')
        self.set_velocitys(self.id_list_hou, leg_vel_hou_stage1)
        flag = True
        while flag:
            positions = rhex.get_positions(rhex.id_list_hou)
            leg_positions = []
            for i in range(2):
                leg_positions.append((positions[2 * i] + positions[2 * i + 1]) / 2 + math.pi / math.pi * 2048)
            # print(leg_positions)
            for i in range(len(leg_positions)):
                # leg_positions[i] = leg_positions[i] % 4096
                if leg_positions[i] >= leg_angle_hou_stage1 / math.pi * 2048:
                    flag = False
                    rhex.set_velocitys(rhex.id_list_hou, [0 for i in range(4)])
                    # print(0)
                    break

        if input('按下任意键开始（e退出）') != 'e':
            pass
        ###################### 阶段二  ###################################
        flag = True
        flag_qian = False
        flag_zhong = False
        flag_hou = False
        duration_stage2 = 3
        leg_angle_hou_stage2 = 2 * math.pi * count + 3.2 * math.pi / 4
        leg_angle_zhong_stage2 = 2 * math.pi * (count - 1) + math.pi + stair_angle
        leg_angle_qian_stage2 = 2 * math.pi * (count - 1) + 4 * math.pi / 5
        leg_vel_hou_stage2 = (leg_angle_hou_stage2 - leg_angle_hou_stage1) / duration_stage2
        leg_vel_zhong_stage2 = (leg_angle_zhong_stage2 - leg_angle_zhong_stage0) / duration_stage2
        leg_vel_qian_stage2 = (leg_angle_qian_stage2 - leg_angle_qian_stage0) / duration_stage2

        # 将所有腿的速度写入列表中
        leg_vel_list = []
        for _ in range(2):
            leg_vel_list.append(leg_vel_qian_stage2)
            leg_vel_list.append(leg_vel_qian_stage2)
            leg_vel_list.append(leg_vel_zhong_stage2)
            leg_vel_list.append(leg_vel_zhong_stage2)
            leg_vel_list.append(leg_vel_hou_stage2)
            leg_vel_list.append(leg_vel_hou_stage2)
        leg_vel_list = leg_vel_list * 2

        # 给所有电机设置速度
        self.set_velocitys(self.id_list, leg_vel_list)

        # 在循环中判断腿是否达到指定位置
        while flag:
            positions = rhex.get_positions(rhex.id_list)
            leg_positions = []
            for i in range(6):
                leg_positions.append((positions[2 * i] + positions[2 * i + 1]) / 2 + math.pi / math.pi * 2048)
            # print(leg_positions)
            if leg_positions[0] >= leg_angle_qian_stage2 / math.pi * 2048 and flag_qian == False:
                flag_qian = True
                rhex.set_velocitys(rhex.id_list_qian, [0] * 4)
            if leg_positions[1] >= leg_angle_zhong_stage2 / math.pi * 2048 and flag_zhong == False:
                flag_zhong = True
                rhex.set_velocitys(rhex.id_list_zhong, [0] * 4)
            if leg_positions[2] >= leg_angle_hou_stage2 / math.pi * 2048 and flag_hou == False:
                flag_hou = True
                rhex.set_velocitys(rhex.id_list_hou, [0] * 4)
            if flag_qian and flag_zhong and flag_hou:
                flag = False
                rhex.set_velocitys(rhex.id_list, [0 for _ in range(12)])

        if input('按下任意键开始（e退出）') != 'e':
            pass
        ######################### 阶段三  ###################################
        flag = True
        flag_qian = False
        flag_zhong = False
        flag_hou = False
        duration_stage3 = 3
        leg_angle_hou_stage3 = 2 * math.pi * count + math.pi
        leg_angle_zhong_stage3 = 2 * math.pi * count + 2 * math.pi / 3
        leg_angle_qian_stage3 = 2 * math.pi * (count - 1) + math.pi + stair_angle
        leg_vel_hou_stage3 = (leg_angle_hou_stage3 - leg_angle_hou_stage2) / duration_stage3
        leg_vel_zhong_stage3 = (leg_angle_zhong_stage3 - leg_angle_zhong_stage2) / duration_stage3
        leg_vel_qian_stage3 = (leg_angle_qian_stage3 - leg_angle_qian_stage2) / duration_stage3

        print("hou", leg_vel_hou_stage3)
        print("zhong", leg_vel_zhong_stage3)
        print("qian", leg_vel_qian_stage3)

        # 将所有腿的速度写入列表中
        leg_vel_list = []
        for _ in range(2):
            leg_vel_list.append(leg_vel_qian_stage3)
            leg_vel_list.append(leg_vel_qian_stage3)
            leg_vel_list.append(leg_vel_zhong_stage3)
            leg_vel_list.append(leg_vel_zhong_stage3)
            leg_vel_list.append(leg_vel_hou_stage3)
            leg_vel_list.append(leg_vel_hou_stage3)
        leg_vel_list = leg_vel_list * 2

        # 给所有电机设置速度
        self.set_velocitys(self.id_list, leg_vel_list)

        # 在循环中判断腿是否达到指定位置
        while flag:
            positions = rhex.get_positions(rhex.id_list)
            leg_positions = []
            for i in range(6):
                leg_positions.append((positions[2 * i] + positions[2 * i + 1]) / 2 + math.pi / math.pi * 2048)
            # print(leg_positions)
            if leg_positions[0] >= leg_angle_qian_stage3 / math.pi * 2048 and flag_qian == False:
                flag_qian = True
                rhex.set_velocitys(rhex.id_list_qian, [0] * 4)
            if leg_positions[1] >= leg_angle_zhong_stage3 / math.pi * 2048 and flag_zhong == False:
                flag_zhong = True
                rhex.set_velocitys(rhex.id_list_zhong, [0] * 4)
            if leg_positions[2] >= leg_angle_hou_stage3 / math.pi * 2048 and flag_hou == False:
                flag_hou = True
                rhex.set_velocitys(rhex.id_list_hou, [0] * 4)
            if flag_qian and flag_zhong and flag_hou:
                flag = False
                rhex.set_velocitys(rhex.id_list, [0 for _ in range(12)])

        if input('按下任意键开始（e退出）') != 'e':
            pass
        ######################### 阶段四  ###################################
        flag = True
        flag_qian = False
        flag_zhong = False
        flag_hou = False
        duration_stage4 = 4
        leg_angle_hou_stage4 = 2 * math.pi * count + leg_angle_hou_stage0
        leg_angle_zhong_stage4 = 2 * math.pi * count + leg_angle_zhong_stage0
        leg_angle_qian_stage4 = 2 * math.pi * count + leg_angle_qian_stage0

        leg_vel_hou_stage4 = (leg_angle_hou_stage4 - leg_angle_hou_stage3) / duration_stage4
        leg_vel_zhong_stage4 = (leg_angle_zhong_stage4 - leg_angle_zhong_stage3) / duration_stage4
        leg_vel_qian_stage4 = (leg_angle_qian_stage4 - leg_angle_qian_stage3) / duration_stage4

        print("hou", leg_vel_hou_stage4)
        print("zhong", leg_vel_zhong_stage4)
        print("qian", leg_vel_qian_stage4)

        # 将所有腿的速度写入列表中
        leg_vel_list = []
        for _ in range(2):
            leg_vel_list.append(leg_vel_qian_stage4)
            leg_vel_list.append(leg_vel_qian_stage4)
            leg_vel_list.append(leg_vel_zhong_stage4)
            leg_vel_list.append(leg_vel_zhong_stage4)
            leg_vel_list.append(leg_vel_hou_stage4)
            leg_vel_list.append(leg_vel_hou_stage4)
        leg_vel_list = leg_vel_list * 2

        # 给所有电机设置速度
        self.set_velocitys(self.id_list, leg_vel_list)

        # 在循环中判断腿是否达到指定位置
        while flag:
            positions = rhex.get_positions(rhex.id_list)
            leg_positions = []
            for i in range(6):
                leg_positions.append((positions[2 * i] + positions[2 * i + 1]) / 2 + math.pi / math.pi * 2048)
            # print(leg_positions)
            if leg_positions[0] >= leg_angle_qian_stage4 / math.pi * 2048 and flag_qian == False:
                flag_qian = True
                rhex.set_velocitys(rhex.id_list_qian, [0] * 4)
            if leg_positions[1] >= leg_angle_zhong_stage4 / math.pi * 2048 and flag_zhong == False:
                flag_zhong = True
                rhex.set_velocitys(rhex.id_list_zhong, [0] * 4)
            if leg_positions[2] >= leg_angle_hou_stage4 / math.pi * 2048 and flag_hou == False:
                flag_hou = True
                rhex.set_velocitys(rhex.id_list_hou, [0] * 4)
            if flag_qian and flag_zhong and flag_hou:
                flag = False
                rhex.set_velocitys(rhex.id_list, [0 for _ in range(12)])

    # 第二种方式爬楼梯
    def climb_stair_walk_V2(self, height, width, count):
        pass

    def finished(self):
        # self.reset_just(self.id_list)
        self.switch_torque(self.id_list, 'disable')
        self.portHandler.closePort()

    def trajectory_generate(self, ladder_width, leg_id):
        pro_vel = 800
        profile_vel = []
        for i in range(len(self.id_list)):
            profile_vel.append([DXL_LOBYTE(DXL_LOWORD(pro_vel)),
                                DXL_HIBYTE(DXL_LOWORD(pro_vel)),
                                DXL_LOBYTE(DXL_HIWORD(pro_vel)),
                                DXL_HIBYTE(DXL_HIWORD(pro_vel))])
        # print("profile_vel%d" %(profile_vel))

        self.write_data(self.id_list, self.ADDR_PRO_PROFILE_VELOCITY, self.LEN_ADDR_PRO_PROFILE_VELOCITY, profile_vel)

        target_pos = [-0.1 * 2 * self.r, -0.6 * 2 * self.r]
        self.legpos_control(target_pos[0], target_pos[1], leg_id)

        if input('按下任意键开始（e退出）') != 'e':
            target_pos[0] = 2.4 * self.r
            target_pos[1] = -0.65 * 2 * self.r
            self.legpos_control(target_pos[0], target_pos[1], leg_id)


    def roll_pos_control1(self,vel_th,idlist,durt):
        t_start=time.time()
        t0=time.time()
        t=0
        N=0
        dposition_raw=self.dt*vel_th*4096/2*math.pi

        data=self.get_positions(idlist)

        real_data_t=[]
        real_data_vel = []
        real_data_pos = []
        real_data_cur=[]

        while t < durt:

            t=time.time()-t0

            if(t>N*self.dt):
                goal_positions=[]
                for i in range(idlist):
                    goal_positions.append(data[i]+N*dposition_raw)

                self.set_positions(goal_positions)

                # real_t=time.time()-t0
                # real_vel=self.get_velocitys(idlist)
                # real_pos=self.get_positions(idlist)
                # real_cur=self.get_currents(idlist)

                # print("t\t",real_t)
                # print("t\t", real_vel)
                # print("t\t", real_pos)
                # print("t\t", real_cur)
                # print("\n")

                # real_data_t.append(real_t)
                # real_data_pos.append(real_pos)
                # real_data_vel.append(real_cur)
                # real_data_cur.append(real_vel)

                N=N+1


if __name__ == '__main__':
    rhex = rhext3(r=0.08, gear_rate=1)
    rhex.reset_offset()
    rhex.offset_position_EXT()#多圈模式
    # rhex.offset_position()#单圈模式
    rhex.init()
    print('当前位置:', rhex.get_positions(rhex.id_list))
    # rhex.reset_just(rhex.id_list)
    # rhex.roll_pos_control1(math.pi/3,[0],3)
    # 爬梯子
    # if input('按下任意键开始（e退出）') != 'e':
    #     rhex.initandclimb(10)

    # 爬楼梯
    # if input('按下任意键开始（e退出）') != 'e':
    #     rhex.climb_stair_init_V1(0.12, 28)

    #爬楼梯
    #if input('按下任意键开始（e退出）') != 'e':
        #rhex.climb_stair_walk_V1(0.12, 28, 1)
    if input('按下任意键开始（e退出）') != 'e':
        # rhex.inittorhex()
        rhex.inittowheel()
    #     rhex.inittoleg()

    if input('按下任意键开始（e退出）') != 'e':
        # rhex.walk_leg(rhex.id_list, durt=20)
    #     rhex.roll_rhex(rhex.id_list, math.pi / 7, 15)
        rhex.roll_vel_control(rhex.id_list, 3*math.pi/5, 15)#轮模式
    # rhex.turn_left(math.pi/4)
    # rhex.turn_left(math.pi/4)
    #    pass

    if input('按下任意键开始（e退出）') != 'e':
        pass
    rhex.finished()

