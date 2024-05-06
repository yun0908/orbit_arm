# 这是一个示例 Python 脚本。

# 按 ⌃R 执行或将其替换为您的代码。
# 按 双击 ⇧ 在所有地方搜索类、文件、工具窗口、操作和设置。

# 将数据限制在范围内
P_MIN = -12.5  # 位置最小值
P_MAX = 12.5  # 位置最大值
V_MIN = -50.0  # 速度最小值
V_MAX = 50.0  # 速度最大值
T_MIN = -65.0  # 扭矩最小值
T_MAX = 65.0  # 扭矩最大值
Kp_MIN = 0  # 比例增益最小值
Kp_MAX = 500.0  # 比例增益最大值
Kd_MIN = 0  # 微分增益最小值
Kd_MAX = 5.0  # 微分增益最大值
I_MAX = 40.0 # 额定电流 或者20A

def float_to_uint(x, x_min, x_max, bits):
    # 将浮点数转换为无符号整数，给定范围和位数
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    return int((x - x_min) * ((1 << bits) / span))

def pack_cmd(msg, p_des, v_des, kp, kd, t_ff):

    p_des = min(max(P_MIN, p_des), P_MAX)
    v_des = min(max(V_MIN, v_des), V_MAX)
    kp = min(max(Kp_MIN, kp), Kp_MAX)
    kd = min(max(Kd_MIN, kd), Kd_MAX)
    t_ff = min(max(T_MIN, t_ff), T_MAX)

    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12)
    kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    msg.data[0] = p_int >> 8
    msg.data[1] = p_int & 0xFF
    msg.data[2] = v_int >> 4
    msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8) # KP 高8位和V低4位组合成data[3]
    msg.data[4] = kp_int & 0xFF # KP低8位存储在data[4]
    msg.data[5] = kd_int >> 4 # KD高8位存储在data[5]
    msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8) # KD低4位和扭矩高8位组合成data[6]
    msg.data[7] = t_int & 0xff # 扭矩低8位存储在data[7]
    for i in range (8):
        msg.data[i] = hex(msg.data[i])
        # print(a)
    for i in range(len(msg.data)):
        if msg.data[i].startswith("0x"):
            msg.data[i] = msg.data[i][2:]

def unpack_reply(msg):
    # 从can缓冲区中解包整数
    id = msg.data[0]  # 驱动ID号
    # print(msg.data[0])
    # print(id)
    p_int = (msg.data[1] << 8) | msg.data[2]
    v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
    i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]
    T_int = msg.data[6]
    # 将整数转换为浮点数
    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    i = uint_to_float(i_int, -I_MAX, I_MAX, 12)
    T = T_int
    postion = p
    speed = v
    torque = i
    Temperature = T - 40
    # 根据ID号读取对应数据，温度范围-40~215
    print(id, postion, speed, torque, Temperature)
    return id, postion, speed, torque, Temperature
    # if id == 1:
    #     postion = p
    #     speed = v
    #     torque = i
    #     Temperature = T - 40
    #     # 根据ID号读取对应数据，温度范围-40~215
    #     print(postion, speed, torque, Temperature)

def uint_to_float(x_int, x_min, x_max, bits):
    # 将无符号整数转换为浮点数，给定范围和位数
    span = x_max - x_min
    offset = x_min
    return ((float(x_int)) * span / ((1 << bits) - 1)) + offset

class Msg():
    # data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    # data_T = []


send_data1 = ["FF", "FF", "FF", "FF", "FF", "FF", "FF", "Fc"]  # 使能
send_data2 = [0x7f, 0xff, 0x82, 0x80, 0x00, 0x66, 0x67, 0xff]  # v1 kd 2
send_data3 = ["7f", "ff", "82", "80", "00", "66", "67", "ff"]  # v-1 kd 2
send_data4 = ["a0", "26", "7f", "f0", "50", "66", "67", "ff"]  # p3.14 kp9.89/9.77 kd 2
send_data5 = ["5f", "d8", "7f", "f0", "50", "66", "67", "ff"]  # p-3.14 kp9.89/9.77 kd 2

unsend_data2 = [0x01, 0x7f, 0xff, 0x82, 0x80, 0xff, 0x00, 0x00]  # v1 kd 2
unsend_data3 = [0x4, 0x89, 0xec, 0x82, 0x78, 0x5, 0x0, 0x0]
unsend_data4 = [4, 137, 236, 130, 120, 5, 0, 0]
# ss = '0x4 0x89 0xec 0x82 0x78 0x5 0x0 0x0'
# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    msg = Msg()
    send = []
    # msg.data=[0x01, 0x7f, 0xff, 0x82, 0x80, 0x66, 0x60, 0xff]
    # for i in range (len(send_data2)):
    #     send.append(float(send_data2[i]))

    msg.data = unsend_data4
    msg.data = [2, 85, 163, 127, 247, 252, 0, 0]
    id, postion, speed, torque, Temperature = unpack_reply(msg)
    # pack_cmd(msg, 0.0, 2.0, 0.0, 2.0, 0.0)
    # print(type(msg.data[0]))
    print(id)