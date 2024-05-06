# --*--utf8--*--
import ctypes
from ctypes import *
import os, sys
import threading
import time

# import Trans

# from main import listreadcan1, listreadcan2

DevType = c_uint

'''
    Device Type
'''
USBCAN1 = DevType(3)
USBCAN2 = DevType(4)
USBCANFD = DevType(6)
'''
    Device Index
'''
DevIndex = c_uint(0)  # 设备索引
'''
    Channel
'''
Channel1 = c_uint(0)  # CAN1
Channel2 = c_uint(1)  # CAN2
'''
    ECAN Status
'''
STATUS_ERR = 0
STATUS_OK = 1

'''
    Device Information
'''


class ERR_INFO(Structure):
    _fields_ = [("ErrCode", c_uint),
                ("Passive_ErrData", c_ubyte * 3),
                ("ArLost_ErrData", c_ubyte)]


class BoardInfo(Structure):
    _fields_ = [("hw_Version", c_ushort),  # 硬件版本号，用16进制表示
                ("fw_Version", c_ushort),  # 固件版本号，用16进制表示
                ("dr_Version", c_ushort),  # 驱动程序版本号，用16进制表示
                ("in_Version", c_ushort),  # 接口库版本号，用16进制表示
                ("irq_Num", c_ushort),  # 板卡所使用的中断号
                ("can_Num", c_byte),  # 表示有几路CAN通道
                ("str_Serial_Num", c_byte * 20),  # 此板卡的序列号，用ASC码表示
                ("str_hw_Type", c_byte * 40),  # 硬件类型，用ASC码表示
                ("Reserved", c_byte * 4)]  # 系统保留


class CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),  # 报文帧ID
                ("TimeStamp", c_uint),  # 接收到信息帧时的时间标识，从CAN控制器初始化开始计时，单位微秒
                ("TimeFlag", c_byte),  # 是否使用时间标识，为1时TimeStamp有效，TimeFlag和TimeStamp只在此帧为接收帧时有意义。
                ("SendType", c_byte),
                # 发送帧类型。=0时为正常发送，=1时为单次发送（不自动重发），=2时为自发自收（用于测试CAN卡是否损坏），=3时为单次自发自收（只发送一次，用于自测试），只在此帧为发送帧时有意义
                ("RemoteFlag", c_byte),  # 是否是远程帧。=0时为数据帧，=1时为远程帧
                ("ExternFlag", c_byte),  # 是否是扩展帧。=0时为标准帧（11位帧ID），=1时为扩展帧（29位帧ID）
                ("DataLen", c_byte),  # 数据长度DLC(<=8)，即Data的长度
                ("data", c_ubyte * 8),  # CAN报文的数据。空间受DataLen的约束
                ("Reserved", c_byte * 3)]  # 系统保留。


class INIT_CONFIG(Structure):
    _fields_ = [("acccode", c_uint32),  # 验收码。SJA1000的帧过滤验收码
                ("accmask", c_uint32),  # 屏蔽码。SJA1000的帧过滤屏蔽码。屏蔽码推荐设置为0xFFFF FFFF，即全部接收
                ("reserved", c_uint32),  # 保留
                ("filter", c_byte),  # 滤波使能。0=不使能，1=使能。使能时，请参照SJA1000验收滤波器设置验收码和屏蔽码
                ("timing0", c_byte),  # 波特率定时器0,详见动态库使用说明书7页
                ("timing1", c_byte),  # 波特率定时器1,详见动态库使用说明书7页
                ("mode", c_byte)]  # 模式。=0为正常模式，=1为只听模式，=2为自发自收模式。


import _ctypes

cwdx = os.getcwd()


class ECAN(object):
    def __init__(self):
        self.dll = cdll.LoadLibrary(cwdx + '/ECanVci64.dll')
        if self.dll == None:
            print("DLL Couldn't be loaded")

    def OpenDevice(self, DeviceType, DeviceIndex):
        try:
            return self.dll.OpenDevice(DeviceType, DeviceIndex, 0)
        except:
            print("Exception on OpenDevice!")
            raise

    def CloseDevice(self, DeviceType, DeviceIndex):
        try:
            return self.dll.CloseDevice(DeviceType, DeviceIndex, 0)
        except:
            print("Exception on CloseDevice!")
            raise

    def InitCan(self, DeviceType, DeviceIndex, CanInd, Initconfig):
        try:
            return self.dll.InitCAN(DeviceType, DeviceIndex, CanInd, byref(Initconfig))
        except:
            print("Exception on InitCan!")
            raise

    def StartCan(self, DeviceType, DeviceIndex, CanInd):
        try:
            return self.dll.StartCAN(DeviceType, DeviceIndex, CanInd)
        except:
            print("Exception on StartCan!")
            raise

    def ReadBoardInfo(self, DeviceType, DeviceIndex):
        try:
            mboardinfo = BoardInfo()
            ret = self.dll.ReadBoardInfo(DeviceType, DeviceIndex, byref(mboardinfo))
            return mboardinfo, ret
        except:
            print("Exception on ReadBoardInfo!")
            raise

    def Receivce(self, DeviceType, DeviceIndex, CanInd, length):
        try:
            recmess = (CAN_OBJ * length)()
            ret = self.dll.Receive(DeviceType, DeviceIndex, CanInd, byref(recmess), length, 0)
            return length, recmess, ret
        except:
            print("Exception on Receive!")
            raise

    def ClearBuffer(self, DeviceType, DeviceIndex, CanInd):
        try:
            return self.dll.ClearBuffer(DeviceType, DeviceIndex, CanInd)
        except:
            print("Exception on ClearBuffer!")
            raise

    def Tramsmit(self, DeviceType, DeviceIndex, CanInd, mcanobj):
        try:
            # mCAN_OBJ=CAN_OBJ*2
            # self.dll.Transmit.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, POINTER(CAN_OBJ),
            # ctypes.c_uint16]
            return self.dll.Transmit(DeviceType, DeviceIndex, CanInd, byref(mcanobj), c_uint16(1))
        except:
            print("Exception on Tramsmit!")
            raise

    def ReadErrInfo(self, DeviceType, DeviceIndex, CanInd):  # 此函数用以获取USBCAN分析仪最后一次错误信息。
        try:
            err_info = ERR_INFO()
            p_err_info = ctypes.pointer(err_info)
            if (self.dll.ReadErrInfo(DeviceType, DeviceIndex, CanInd, p_err_info) == 1):
                print("ErrCode：", p_err_info.contents.ErrCode)
                print("Passive ErrData：", p_err_info.contents.Passive_ErrData[:])
                print("ArLost  ErrData：", p_err_info.contents.ArLost_ErrData)
                # return ret
        except:
            print("Exception on ReadErrInfo!")
            raise


# 加载动态库
ecan = ECAN()

# CAN 相关参数
musbcanopen = False
rec_CAN1 = 1
rec_CAN2 = 1


#  读取数据
def ReadCAN():
    global musbcanopen, rec_CAN1, rec_CAN2
    if (musbcanopen == True):
        scount = 0
        while (scount < 50):
            scount = scount + 1
            len, rec, ret = ecan.Receivce(USBCAN2, DevIndex, Channel1, 1)
            if (len > 0 and ret == 1):
                mstr = "Rec: " + str(rec_CAN1)
                rec_CAN1 = rec_CAN1 + 1
                if rec[0].TimeFlag == 0:
                    mstr = mstr + " Time: "
                else:
                    mstr = mstr + " Time:" + hex(rec[0].TimeStamp).zfill(8)
                if rec[0].ExternFlag == 0:
                    mstr = mstr + " ID:" + hex(rec[0].ID).zfill(3) + " Format:Stand "
                else:
                    mstr = mstr + " ID:" + hex(rec[0].ID).zfill(8) + " Format:Exten "
                if rec[0].RemoteFlag == 0:
                    mstr = mstr + " Type:Data " + " Data: "
                    for i in range(0, rec[0].DataLen):
                        mstr = mstr + hex(rec[0].data[i]).zfill(2) + " "
                else:
                    mstr = mstr + " Type:Romte " + " Data: Remote Request"

                print("CAN1已收到数据：", mstr)
            len2, rec2, ret2 = ecan.Receivce(USBCAN2, DevIndex, Channel2, 1)
            if (len2 > 0 and ret2 == 1):
                mstr = "Rec: " + str(rec_CAN2)
                rec_CAN2 = rec_CAN2 + 1
                if rec2[0].TimeFlag == 0:
                    mstr = mstr + " Time: "
                else:
                    mstr = mstr + " Time:" + hex(rec2[0].TimeStamp).zfill(8)
                if rec2[0].ExternFlag == 0:
                    mstr = mstr + " ID:" + hex(rec2[0].ID).zfill(3) + " Format:Stand "
                else:
                    mstr = mstr + " ID:" + hex(rec2[0].ID).zfill(8) + " Format:Exten "
                if rec2[0].RemoteFlag == 0:
                    mstr = mstr + " Type:Data " + " Data: "
                    for i in range(0, rec2[0].DataLen):
                        mstr = mstr + hex(rec2[0].data[i]).zfill(2) + " "
                else:
                    mstr = mstr + " Type:Romte " + " Data: Remote Request"

                print("CAN1已收到数据：", mstr)

        t = threading.Timer(0.03, ReadCAN)
        t.start()


t = threading.Timer(0.03, ReadCAN)


# python调用动态库默认参数为整型
def caninit(mbuad):
    global musbcanopen, t, rec_CAN1, rec_CAN2
    if (musbcanopen == False):
        initconfig = INIT_CONFIG()
        initconfig.acccode = 0  # 设置验收码
        initconfig.accmask = 0xFFFFFFFF  # 设置屏蔽码
        initconfig.filter = 0  # 设置滤波使能

        # 波特率设定
        mbaudcan1 = mbuad
        # 打开设备
        if (ecan.OpenDevice(USBCAN2, DevIndex) != STATUS_OK):
            print("ERROR, OpenDevice Failed!")
            return
        initconfig.timing0, initconfig.timing1 = getTiming(mbaudcan1)
        initconfig.mode = 0
        # 初始化CAN1
        if (ecan.InitCan(USBCAN2, DevIndex, Channel1, initconfig) != STATUS_OK):
            print("ERROR, InitCan CAN1 Failed!")
            ecan.CloseDevice(USBCAN2, DevIndex)
            return

        if (ecan.StartCan(USBCAN2, DevIndex, Channel1) != STATUS_OK):
            print("ERROR, StartCan CAN1 Failed!")
            ecan.CloseDevice(USBCAN2, DevIndex)
            return

        musbcanopen = True
        rec_CAN1 = 1
        rec_CAN2 = 1
        print("开启CAN1通信成功")

        t = threading.Timer(0.03, ReadCAN)
        t.start()


    else:
        musbcanopen = False
        ecan.CloseDevice(USBCAN2, DevIndex)


# 拿到波特率
def getTiming(mbaud):
    if mbaud == "1M":
        return 0, 0x14
    if mbaud == "800k":
        return 0, 0x16
    if mbaud == "666k":
        return 0x80, 0xb6
    if mbaud == "500k":
        return 0, 0x1c
    if mbaud == "400k":
        return 0x80, 0xfa
    if mbaud == "250k":
        return 0x01, 0x1c
    if mbaud == "200k":
        return 0x81, 0xfa
    if mbaud == "125k":
        return 0x03, 0x1c
    if mbaud == "100k":
        return 0x04, 0x1c
    if mbaud == "80k":
        return 0x83, 0xff
    if mbaud == "50k":
        return 0x09, 0x1c


def clearcan1():
    listreadcan1.delete(0, END)


def clearcan2():
    listreadcan2.delete(0, END)


def sendcan1(ID_CAN1, len_sdata, send_data):
    global musbcanopen
    if (musbcanopen == False):
        print("ERROR, 请先打开设备")
    else:
        information = send_data
        canobj = CAN_OBJ()
        canobj.ID = int(ID_CAN1, 16)
        canobj.DataLen = int(len_sdata)
        for i in range(len_sdata):
            canobj.data[i] = int(information[i], 16)

        canobj.RemoteFlag = 0
        canobj.ExternFlag = 0
        ecan.Tramsmit(USBCAN2, DevIndex, Channel1, canobj)


# ------------------------------- 以下开始 -----------------------------------------------
caninit_set = "1M"
caninit(caninit_set)

# readmess()

# ID_CAN1 = ["1", "4", "5", "6"]
# ID_CAN1 = ["4"]
'''
    CAN ID 设定
'''
# ID_CAN1 = ["1", "2", "3", "4"]
ID_CAN1 = ["1", "2", "3", "4"]

'''
    白电机 指令集
'''
send_data_POS1 = ["44", "00", "60", "00", "00"]  # position1
send_data_POS2 = ["44", "00", "40", "19", "00"]  # position2
send_data_POS3 = ["43", "00", "60", "00", "00"]  # position1
send_data_POS4 = ["43", "00", "40", "19", "00"]  # position2
send_data_speed1 = ["1d", "e8", "fc", "ff", "ff"]  # v1    反转
send_data_speed2 = ["1d", "e8", "03", "00", "00"]  # v2    正转
send_data_speed3 = ["1d",  "00", "00", "e8", "20"]  # v3    正转
send_data_disable = ["02"]  # 停止

send_speed1 = ["24", "e8", "03", "00", "00"]
send_speed2 = ["25", "08", "fc", "ff", "ff"]
read_speed1 = ["18"]
read_speed2 = ["19"]
read_mod = ["03"]
len_send_stop = len(send_data_disable) #1
len_send_move = len(send_data_POS1)     #5
len_send_ID = len(ID_CAN1)

t = threading.Timer(0.03, ReadCAN)
t.start()

'''
    发送指令
'''
# 失能
for i in range(len_send_ID):
    sendcan1(ID_CAN1[i], len_send_stop, send_data_disable)
    ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)


#设置最大正向允许速度
for i in range(len_send_ID):
    sendcan1(ID_CAN1[i], len_send_move, send_speed1)
    sendcan1(ID_CAN1[i], len_send_stop, read_speed1)
    ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)
    time.sleep(0.5)

#设置最小负向允许速度
for i in range(len_send_ID):
    sendcan1(ID_CAN1[i], len_send_move, send_speed2)
    sendcan1(ID_CAN1[i], len_send_stop, read_speed2)
    ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)
    time.sleep(0.5)
# #读取允许速度
# for i in range(len_send_ID):
#     sendcan1(ID_CAN1[i], len_send_stop,read_speed2)
#     ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)
# for i in range(len_send_ID):
#     sendcan1(ID_CAN1[i], len_send_stop,read_speed1)
#     ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)


# Move
for i in range(10):
    if (i % 2 == 0):
        ss = send_data_POS1
    else:
        ss = send_data_POS2
    for j in range(len_send_ID):
        sendcan1(ID_CAN1[j], len_send_move, ss)
        time.sleep(1)
        # ReadCAN()
        ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)

    # ecan.ReadErrInfo(USBCAN2, DevIndex, Channel1)
    print("hah")

# 失能
for i in range(len_send_ID):
    sendcan1(ID_CAN1[i], len_send_stop, send_data_disable)
    ecan.ClearBuffer(USBCAN2, DevIndex, Channel1)