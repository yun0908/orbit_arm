import time
import math
import serial
import serial.tools.list_ports
import array
import binascii
import crcmod
from time import sleep
from binascii import *

# 电机一个脉冲大概为1.5um


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
def EleRead_Location():
    data = ser.read(4)  # 读四个字节指令
    data_hex = str(binascii.b2a_hex(data))[2:-1]
    # print(data_hex)
    if data_hex == Location_Success:
        print("数据发送成功")
    else:
        print("数据发送失败")
    ser.flushInput()  # 清除输入缓存区数据


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
    EleRead_Location()


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


# 读取电机当前位置的函数，当前读取的是 相对位置
def EleAbsolute_Read():
    data = "01366b"
    ser.write(bytes.fromhex(data))

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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # CW为正方向
    GetAllSerial()  # 检查可用串口
    port_open_recv()  # 打开串口
    #EleMove_Send("CW", 500, 0, 10, "rel")
    EleTurnZero()
    sleep(1)  # 起到一个延时的效果，这里如果不加上一个while True，程序执行一次就自动跳出了
    ser.close()
