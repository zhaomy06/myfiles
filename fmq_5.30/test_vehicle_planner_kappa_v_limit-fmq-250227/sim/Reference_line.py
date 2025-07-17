import cubic_spline as cs
import math


class Reference_line:
    ref_x = []
    ref_y = []
    ref_yaw = []
    ref_k = []
    ref_s = []
    step = 0

    def __init__(self, x_init, y_init, step):
        self.step = step
        self.ref_x, self.ref_y, self.ref_yaw, self.ref_k, self.ref_s = cs.calc_spline_course(
            x_init, y_init, ds=step)


class Reference_line_for_speed_planner:
    ref_x = []
    ref_y = []
    ref_yaw = []
    ref_s = []

    def __init__(self, x, y, yaw,kappa):
        self.ref_x = x
        self.ref_y = y
        self.ref_yaw = yaw
        self.ref_k = kappa
        self.ref_s=[]                   #这个bug太奇怪了，如果没有这个初始化，ref_s的表现会像个静态变量一样
        ss=0.0
        for i in range(0, len(x)):
            #print("第", i, "个s被计算出来")
            if i == 0:
                self.ref_s.append(0)
            else:
                self.ref_s.append(math.sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2) + ss)
                ss+=math.sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2)
