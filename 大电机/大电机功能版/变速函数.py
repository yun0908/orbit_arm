def sendtime(ID, time, acceleration):   #时间，加速度
    global musbcanopen
    if (musbcanopen == True):
        scount = 0
        mstr = []
        t = time            #s
        a = acceleration    #du/s^2
        id = ID
        while (scount < 50):
            scount = scount + 1
            len, rec, ret = ecan.Receivce(USBCAN2, DevIndex, Channel1, 1)
            if (len > 0 and ret == 1):
                for i in range(4, rec[0].DataLen):
                    mstr.append(int(hex(rec[0].data[i]), 16))
                res = mstr[0] + (mstr[1] << 8) + (mstr[2] << 16) + (mstr[3] << 24)
                position = (res / 6553600) * 360
                print("当前位置：", position)
                s = 0.5 * a * t ** 2
                goal_position = (s + position) * 6553600 / 360
                RxData = [44, 0, 0, 0, 0]
                RxData[1] = goal_position & 0xFF
                RxData[2] = (goal_position >> 8) & 0xFF
                RxData[3] = (goal_position >> 16) & 0xFF
                RxData[4] = (goal_position >> 24) & 0xFF
                string_data = [str(num) for num in RxData]
                sendcan1(id, 5, string_data)
        t = threading.Timer(0.03, ReadCAN)
        t.start()