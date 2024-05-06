def com2neg_num(num=None):
    if isinstance(num, int) or isinstance(num, float):
        mode = 2 ** 16
        return -(mode - num)
    else:
        raise TypeError("参数类型错误，形参类型应为'int' or 'float'")


if __name__ == '__main__':
    current = com2neg_num("65367")
    print(current)
