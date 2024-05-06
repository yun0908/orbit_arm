from .dynamixel_control_X import *
from .robot_control import *
from .Gravity_compensation_error_10_V3 import *
from .Visual_Verification import *
# from 四连杆运动学.fkine_fourLink import *


def pos2rad(arr):
    arr_ = arr
    if isinstance(arr_, np.ndarray):
        return ((arr_ - 2048) / 2048) * pi
    else:
        arr_ = np.array(arr_)
        return ((arr_ - 2048) / 2048) * pi


def deg2rad(arr):
    arr_ = arr
    if isinstance(arr_, np.ndarray):
        return arr_ * (pi / 180)
    else:
        arr_ = np.array(arr_)
        return arr_ * (pi / 180)


def rad2deg(arr):
    arr_ = arr
    if isinstance(arr_, np.ndarray):
        return arr_ * (180 / pi)
    else:
        arr_ = np.array(arr_)
        return arr_ * (180 / pi)
