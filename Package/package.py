
#from ast import Pass
#import profile
#from re import X
import sys
import os
import time
import math
#from tkinter import CURRENT
#from wsgiref import validate
#import matplotlib.pyplot as plt
# from spatialmath import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library

#import pandas as pd
import numpy as np


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

'''class fkine:
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

        return xd, yd'''
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
    ADDR_PID_P = 84

    # Data Byte Length
    LEN_PRO_GOAL_POSITION = 4
    LEN_PRO_PRESENT_POSITION = 4
    LEN_PRO_GOAL_VELOCITY = 4
    LEN_PRO_PRESENT_VELOCITY = 4
    LEN_PRO_PRESENT_CURRENT = 2
    LEN_PRO_HOMING_OFFSET = 4
    LEN_PRO_PRESENT_VOLTAGE = 2
    LEN_PRO_PROFILE_VELOCITY = 4
    LEN_PID_P = 2

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
    # DXL_MOVING_STATUS_THRESHOLD = 1  # Dynamixel moving status threshold

    # mode
    ADDR_OPERATING_MODE = 11
    EXT_POSITION_CONTROL_MODE = 4
    VELOCITY_MODE = 1
    POSITION_MODE = 3
    CURRENT_BASED_POSITION_CONTROL_MODE = 5

    index = 0
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

    def __init__(self, r, gear_rate):
        # 我的参数     改
        # 三�?形定义：那一侧上�?��，就说是那一侧三角形
        self.id_list_1 = [0, 1]
        self.id_list = [i for i in range(12)]
        self.id_list_left = [2, 3, 6, 7, 10, 11]
        self.id_list_right = [0, 1, 4, 5, 8, 9]
        self.dt = 0.02

        # 机器人的模式
        self.wheel_mode = 0
        self.rhex_mode = 1
        self.leg_mode = 2

        # 腿模式行走的一些参数
        self.walk_len = 0.1
        self.step_H = -2.0 * r  # 腿末端到转轴的垂直高度
        self.step_h = 0.9 * r  # 腿行走时抬起的高度
        self.walk_vel = 0.08

        self.gear_rate = gear_rate
        self.r = r
        self.ikine = ikine(r)
        #self.fkine = fkine(r)

        # self.curmove = 'init' # 这样改不好，因为如果一�?��令�?一�?��令，会�?覆盖
        self.curmotormode = 'extend_position'
        self.currobotmode = 'init'

        # rhex运动参数
        self.fai_land=math.pi/3
        self.fai_land=math.pi-1*fai_land/2
        self.fai_land=math.pi-1*fai_land/2
        '''
        # 创建处理数据传输的�?�?
        '''
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        #初始化数据字典
        keys=['data_t', 'data_pos', 'data_vel', 'data_cur', 'data_vol']
        self.data_dict = dict(zip(keys, ([] for _ in keys)))

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
        # 开关力矩
            输入：若干电机编号（列表）；开或关信号（字符串，enable、disable）
            功能：开关电机的力矩
        '''

    def switch_torque(self, id_list, switch):

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
        # 切换电机的控制模式?
            输入：若干电机编号（列表）；模式名字（字符串，三种模式：position、extend_position、velocity）
            注意：切换模式前要先把电机力矩关掉才能换模式。结束后再打开力矩
        '''

    def switch_mode(self, id_list, mode_name):

        self.switch_torque(id_list, 'disable')

        if mode_name == 'position':
            mode = self.POSITION_MODE
        elif mode_name == 'extend_position':
            mode = self.EXT_POSITION_CONTROL_MODE
        elif mode_name == 'velocity':
            mode = self.VELOCITY_MODE
        elif mode_name == 'current-base position':
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
    
        # 初始化电机，把所有电机换到扩展位置模式
        '''

    def init(self):

        self.switch_torque(self.id_list, "disable")
        # self.write_data(self.id_list_1,44,4,[262,262]) #修改位置模式情况下的速度极限 需要在力矩关闭情况下使用
        # print(len(self.id_list_1))
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

    '''
        # 同�?读写电机的存储器上的数据
            输入：若干电机编号（列表）；要�?写的位置的地址（标量，预定义）；�?读写的位�?��地址长度（标量，预定义）
            输出：各�?��机内对应地址的值（列表�?/�?
        '''

    def read_data(self, id_list, adr, adr_len):
        if isinstance(id_list, list):
            # 造卡车
            groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, adr, adr_len)

            # 告诉卡车有哪些货站
            for id in id_list:
                dxl_addparam_result = groupSyncRead.addParam(id)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncRead addparam failed" % id)
                    quit()

            # 卡车出发
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # 看各站是否有货物
            for id in id_list:
                dxl_getdata_result = groupSyncRead.isAvailable(id, adr, adr_len)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % id)
                    quit()
                    # sys.exit()
            # 卡车装货，读取数据
            data = []
            for id in id_list:
                data.append(groupSyncRead.getData(id, adr, adr_len))
            # 卡车是局部变量，出去之后自动销毁，不用清空
            return data
        else:
            # Read present position
            if adr_len == 1:
                data, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id_list, adr)
            elif adr_len == 2:
                data, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id_list, adr)
            elif adr_len == 4:
                data, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id_list, adr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return data


    def write(self, id_list, adr, adr_len, data):
        if isinstance(id_list, list):
            # 造卡车
            groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, adr, adr_len)

            # 告诉卡车有哪些货站
            for i in range(len(id_list)):
                dxl_addparam_result = groupSyncWrite.addParam(id_list[i], data[i])
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % i)
                    quit()

            # 卡车出发到各站，并卸货
            dxl_comm_result = groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # 卡车是局部变量，出去之后自动销毁，不用清空

            return
        else:
            if adr_len == 1:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id_list, adr, data)
            elif adr_len == 2:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id_list, adr, data)
            elif adr_len == 4:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id_list, adr, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write_data(self, id_list, adr, adr_len, data):
        if isinstance(id_list, list):
            param_data = []
            for i in range(len(id_list)):
                param_data.append([DXL_LOBYTE(DXL_LOWORD(data[i])),
                                             DXL_HIBYTE(DXL_LOWORD(data[i])),
                                             DXL_LOBYTE(DXL_HIWORD(data[i])),
                                             DXL_HIBYTE(DXL_HIWORD(data[i]))])
            self.write(id_list, adr, adr_len, param_data)
        else:
            self.write(id_list, adr, adr_len,data)

    def set_pid_p(self, id_list, goal_pid_p):
        if isinstance(id_list, list):
            if not isinstance(goal_pid_p,list):
                goal_pid_p=[goal_pid_p]*len(id_list)

            for i in range(len(goal_pid_p)):
                goal_pid_p[i] = int(round(goal_pid_p[i]))
            self.write_data(id_list, self.ADDR_PID_P, self.LEN_PID_P, goal_pid_p)
            return
        else:
            goal_pid_p=int(round(goal_pid_p))
            self.write_data(id_list, self.ADDR_PID_P, self.LEN_PID_P, goal_pid_p)
# 获取电机位置
    def get_positions(self, id_list):
        positions = self.read_data(id_list, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
        if isinstance(positions, list):
            # 这里为了解决溢出问题?
            for i in range(len(positions)):
                if positions[i] > 2 ** 31:
                    positions[i] -= 2 ** 32
            return positions
        else:
            if positions > 2 ** 31:
                positions -= 2 ** 32
            return positions
    # 获取电机速度
    def get_velocitys(self, id_list):
        velocitys = self.read_data(id_list, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_PRESENT_VELOCITY)

        # 这里为了解决溢出问题?
        for i in range(len(velocitys)):
            if velocitys[i] > 2 ** 31:
                velocitys[i] -= 2 ** 32
        return velocitys

    # 获取电机电流
    def get_currents(self, id_list):
        currents = self.read_data(id_list, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

        # 这里为了解决溢出问题?
        for i in range(len(currents)):
            if currents[i] > 2 ** 15:
                currents[i] = currents[i] - 2 ** 16
            currents[i] = round(currents[i] * 2.69 / 1000, 3)        #电流单位2.69mA，保留3位小数
        return currents

    # 获取电机电压
    def get_voltage(self, id_list):
        voltage = self.read_data(id_list, self.ADDR_PRO_PRESENT_VOLTAGE, self.LEN_PRO_PRESENT_VOLTAGE)

        for i in range(len(voltage)):
            if voltage[i] > 2 ** 15:
                voltage[i] = 2 ** 16 - voltage[i]
        for i in range(len(voltage)):           #电压单位是0.1，需要转化
            voltage[i] = round(voltage[i] * 0.1, 1)  #round 强制保留一位小数
        return voltage

    def print_allall(self, id_list):
        print('cur_pos:\t', self.get_positions(id_list))
        print('cur_vel:\t', self.get_velocitys(id_list))
        print('cur_cur:\t', self.get_currents(id_list))

        return



    '''
        # 设置每个电机的应到指定位置
            输入：可以输入多个电机（列表），也可以输入单个电机值，目标位置也可以是列表或单个值
        '''

    def set_positions(self, id_list, goal_positions):
        if isinstance(id_list, list):
            if not isinstance(goal_positions,list):
                goal_positions=[goal_positions]*len(id_list)

            for i in range(len(goal_positions)):
                goal_positions[i] = int(round(goal_positions[i]))
            self.write_data(id_list, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION, goal_positions)
            return
        else:
            goal_positions=int(round(goal_positions))
            self.write_data(id_list, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION, goal_positions)

        '''
           # 让电机在位置模式下以指定速度旋转
               输入：可以输入多个电机（列表），也可以输入单个电机值，
                    目标速度可以为单个值或者列表
                    运动时间
    '''
    def roll_pos_control(self,id_list,vel_th,durt):
        t_start=time.time()
        print('>>>>> roll_pos_control() Start!')
        if isinstance(vel_th, list):
            vel_th_islist=True
            dposition_raw_list=[0]*len(vel_th)
            for i in range(len(vel_th)):
                dposition_raw_list[i]=vel_th[i] * self.dt * 4096 / (2 * math.pi)
        else:
            vel_th_islist=False
            dposition_raw = vel_th * self.dt * 4096 / (2 * math.pi)

        data=self.get_positions(id_list)
        t0 = time.time()
        t = 0
        N = 0

        while t < durt:
            t = time.time() - t0
            if t > N * self.dt:
                if isinstance(id_list,list):
                    if vel_th_islist:
                        goal_positions = []
                        for i in range(len(data)):
                            goal_positions.append(int(data[i] + N * dposition_raw_list[i]))  # 注意这里int;不用注意了，我写在函数里了，需要四舍五入的int
                    else:
                        goal_positions = []
                        for i in range(len(data)):
                            goal_positions.append(int(data[i] + N * dposition_raw))  # 注意这里int;不用注意了，我写在函数里了，需要四舍五入的int
                else:
                    goal_positions=int(data + N * dposition_raw)

                self.set_positions(id_list, goal_positions)
                N = N + 1

        print('^^^^^ roll_pos_control() Finished!')
        print('^^^^^ Time_Consumed: ', time.time() - t_start)

        return
    '''
        用于读多个电机值
            输入: data：用于存放数据,类型为字典
                list:  要读取的电机id 类型为列表
                durt: 读取时间,默认值为0.1
    '''
    def record_save(self, data: dict, id_list: list, t_start, durt=0.1):
        sample = 0
        t0 = time.time()
        while time.time() - t0 < durt:
            if time.time() - t0 > sample * 0.10:
                sample = sample + 1
                read_t = time.time() - t_start
                read_pos = self.get_positions(id_list)
                # read_vel = self.get_velocitys(id_list)
                # read_cur = self.get_currents(id_list)
                # read_vol = self.get_voltage(id_list)
                data['data_t'].append(read_t)
                data['data_pos'].append(read_pos)
                # data['data_vel'].append(read_vel)
                # data['data_cur'].append(read_cur)
                # data['data_vol'].append(read_vol)
        # return data

    def set_same_target_position(self,id_list,same_target_position):
        pass

    def set_allarrive_target_position(self, id_list, target_positions):
        pass

    def set_velocitys(self,id_list,goal_velocitys):


        if isinstance(id_list,list):
            # switch_mode(id_list, 'velocity') 要用这个函数，就要在那个模块上加上这�?
            # 这个函数不能单独使用，因为�?考虑到模式转换的�??
            if not isinstance(goal_velocitys, list):
                goal_velocitys = [goal_velocitys] * len(id_list)
            goal_velocitys_trans = []
            for i in range(len(goal_velocitys)):
                goal_velocitys_trans.append(int(round(goal_velocitys[i] * 60 / (2 * math.pi * 0.229))))
                print(i,':',goal_velocitys_trans[i])
            # print(goal_velocitys_trans)
            self.write_data(id_list, self.ADDR_PRO_GOAL_VELOCITY, self.LEN_PRO_GOAL_VELOCITY, goal_velocitys_trans)
            return
        else:
            goal_velocitys=int(round(goal_velocitys * 60 / (2 * math.pi * 0.229)))
            self.write_data(id_list,self.ADDR_PRO_GOAL_VELOCITY,self.LEN_PRO_GOAL_VELOCITY,goal_velocitys)

    def roll_vel_control(self, id_list, vel, durt):

        t_start = time.time()
        print('==========')
        print('>>>>> roll_vel_control() Start!')

        self.switch_mode(id_list, 'velocity')
        self.set_velocitys(id_list, vel)

        t0 = time.time()
        t = 0
        while t < durt:
            t = time.time() - t0

        self.set_velocitys(id_list, 0)

    def roll_rhex(self, vel_land, durt):

        t_start = time.time()
        print('==========')
        print('>>>>> roll_rhex() Start!')

        goal_positions=self.get_positions(self.id_list)
        t_inland=self.fai_land/vel_land
        vel_lift=((2*math.pi-self.fai_land)/((t_inland/0.5)*0.5))

        dposition_land=vel_land*self.dt*4096/(2*math.pi)
        dposition_lift=vel_lift*self.dt*4096/(2*math.pi)

        t0=time.time()
        t=0
        N=0
        sample=0

        while t<durt:
            t=t-time.time()-t0
            if t>N*self.dt:
                leg_positions=[]
                for i in range(0,12,2):
                    leg_positions.append((goal_positions[i]+goal_positions[i+1])/2+math.pi / math.pi * 2048)

                for i in range(len(leg_positions)):
                    leg_positions[i]=leg_positions[i]%(4096)
                    if (self.fail_in_land/math.pi*2048)<=leg_positions[i]<=(self.fail_out_land/math.pi*2048):
                        goal_positions[i*2]=goal_positions[i*2]+dposition_land
                        goal_positions[i*2+1]=goal_positions[i*2+1]+dposition_land
                    else:
                        goal_positions[i*2]=goal_positions[i*2]+dposition_lift
                        goal_positions[i*2+1]=goal_positions[i*2+1]+dposition_lift
                self.set_positions(self.id_list,goal_positions)

                N=N+1
                if t > sample * 0.15:
                    sample = sample + 1
                    # 读的都是电机的�?
                    # read_t = time.time() - t0
                    # #read_pos = self.get_positions(id_list)
                    # #read_vel = self.get_velocitys(id_list)
                    # read_cur = self.get_currents(id_list)
                    # read_vol = self.get_voltage(id_list)
                    # print('delta_t:\t', read_t)
                    # print('cur_pos:\t', read_pos)
                    # print('cur_pos:\t', read_vol)
                    # print('cur_vel:\t', read_vel)f
                    # print('cur_cur:\t', read_cur)
                    # read_data_t.append(read_t)
                    # #read_data_pos.append(read_pos)
                    # #read_data_vel.append(read_vel)
                    # read_data_cur.append(read_cur)
            # df_vel = pd.DataFrame(read_data_cur, read_data_t, columns=range(len(id_list)))
            # df_vel.to_csv('./data/rhex/rhex_cur')
            print('^^^^^ roll_rhex() Finished!')
            print('^^^^^ Time_Consumed: ', time.time() - t_start)

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

    def finished(self):
        # self.reset_just(self.id_list)
        self.switch_torque(self.id_list_1, 'disable')
        self.portHandler.closePort()


















if __name__ == '__main__':
    rhex = rhext3(r=0.08, gear_rate=1)
    rhex.switch_torque(rhex.id_list_1, 'enable')
    rhex.switch_mode(rhex.id_list_1, 'extend_position')
    # print(rhex.read_data(1,rhex.ADDR_PRO_GOAL_POSITION,rhex.LEN_PRO_GOAL_POSITION))
    goal_position = [4500,3600]
    #rhex.write_data(rhex.id_list_1,rhex.ADDR_PRO_GOAL_POSITION,rhex.LEN_PRO_GOAL_POSITION,goal_position)
    # rhex.set_positions(rhex.id_list_1, [4000,2000])
    # time.sleep(5)
    # print(rhex.get_positions(rhex.id_list_1))
    # rhex.roll_pos_control(rhex.id_list_1, [math.pi/2,math.pi],4)
    # rhex.record_save(rhex.data_dict,rhex.id_list_1,time.time(),4)
    #
    # tim=rhex.data_dict['data_t']
    # pos=rhex.data_dict['data_pos']
    #
    # for i in range(len(tim)):
    #     print("data_t:",tim[i],'data_pos:',pos[i])
    # rhex.switch_mode(rhex.id_list_1, 'velocity')
    # rhex.write_data(0, rhex.ADDR_PRO_GOAL_VELOCITY, rhex.LEN_PRO_GOAL_VELOCITY, )
    # rhex.set_velocitys(rhex.id_list_1,[2*math.pi,2*math.pi])
    # rhex.roll_vel_control(rhex.id_list_1,[math.pi,math.pi*2],4)
    # print(rhex.read_data(1,44,4))
    time.sleep(5)
    rhex.finished()








