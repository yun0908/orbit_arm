x=int(input("四字节求速度，请输入1；速度求四字节，请输入2！"))
def calculate_rx_data(angle):
    res = int(angle)

    RxData = [0, 0, 0, 0, 0]
    RxData[1] = res & 0xFF
    RxData[2] = (res >> 8) & 0xFF
    RxData[3] = (res >> 16) & 0xFF
    RxData[4] = (res >> 24) & 0xFF

    return RxData


if x==1:
# 依次输入四个十六进制数
    RxData = []  # 存储输入的四个十六进制数
    for i in range(4):
        hex_str = input("请输入第{}个十六进制数: ".format(i+1))
        RxData.append(int(hex_str, 16))  # 将十六进制字符串转换为整数并添加到列表中

# 计算res的值
    res = RxData[0] + (RxData[1] << 8) + (RxData[2] << 16) + (RxData[3] << 24)

    print("速度的值为:", res)


elif x==2:
# 输入减速机角度和减速比
    angle = float(input("请输入十进制速度："))


    RxData = calculate_rx_data(angle)
    print("RxData[1] =", hex(RxData[1]))
    print("RxData[2] =", hex(RxData[2]))
    print("RxData[3] =", hex(RxData[3]))
    print("RxData[4] =", hex(RxData[4]))

else:
    print("输入错误")