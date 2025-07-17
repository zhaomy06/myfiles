import math
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from Dynamics_model import Dy_Car
from pid_lon import PID
from Kinematics_model import Ki_Car

PI = math.pi


class LQR_controller:
    def __init__(self):
        self.lf = 1.0
        self.lr = 1.9
        self.m = 1412
        self.L = self.lf + self.lr
        self.Cf = -112600
        self.Cr = -89500
        self.i = 0
        self.x = 0
        self.y = 0
        self.v = 10
        self.vx = 0
        self.timestamp = 0
        self.phi = 0
        self.steer = 0
        self.K = np.zeros((3000, 4))
        self.tar_xlist = None
        self.tar_ylist = None
        self.tar_hlist = None
        self.tar_vlist = None
        self.tar_klist = None
        self.lqr_mode = 0
        self.norepeat_phi_list = []
        self.norepeat_t_list = []
        self.ed = 0
        self.d_ed = 0
        self.ephi = 0
        self.d_ephi = 0
        self.match_x = 0
        self.match_y = 0
        self.match_h = 0
        self.match_v = 0
        self.match_k = 0
        self.u_feedback = 0
        self.u_feedforward = 0
        self.u = 0
        self.KDTree = None
        self.ed_list = []
        self.d_ed_list = []
        self.e_phi_list = []
        self.d_e_phi_list = []
        self.u_feedback_list = []
        self.u_feedforward_list = []

    def LQR_Init(self):
        self.K = np.loadtxt('K.txt')

    def NormalizeAngle(self, angle):
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle < - math.pi:
            angle += 2 * math.pi
        return angle

    def close_point(self, car_model):
        car_state = np.zeros(2)
        car_x = car_model.x
        car_y = car_model.y
        car_state[0] = car_x
        car_state[1] = car_y
        _, index = self.KDTree.query([car_x, car_y])
        self.i = index

    def cal_k(self, x_list, y_list):
        k_list = [0]
        for i in range(1, len(x_list) - 1):
            a_index = i - 1
            b_index = i
            c_index = i + 1
            d_a = math.sqrt(pow(x_list[b_index] - x_list[c_index], 2) + pow(y_list[b_index] - y_list[c_index], 2))
            d_b = math.sqrt(pow(x_list[a_index] - x_list[c_index], 2) + pow(y_list[a_index] - y_list[c_index], 2))
            d_c = math.sqrt(pow(x_list[b_index] - x_list[a_index], 2) + pow(y_list[b_index] - y_list[a_index], 2))
            if d_a < 0.01:
                d_a = 0.01
            if d_c < 0.01:
                d_c = 0.01
            cos = (d_a * d_a + d_c * d_c - d_b * d_b) / (2 * d_a * d_c)
            if (1 - cos * cos) < 0:
                k_temp = 0.01
            else:
                sin = math.sqrt(1 - cos * cos)
                k_temp = 2 * sin / d_b
            k_list.append(k_temp)
        k_list.append(0)
        return k_list

    def update_trajectory(self, x_list, y_list, h_list, v_list):
        self.tar_xlist = x_list
        self.tar_ylist = y_list
        self.tar_hlist = h_list
        self.tar_vlist = v_list
        self.tar_klist = self.cal_k(x_list, y_list)
        self.tar_klist[1] = 0.001

        refer_path = np.zeros((len(self.tar_xlist), 2))
        refer_path[:, 0] = self.tar_xlist
        refer_path[:, 1] = self.tar_ylist
        self.KDTree = KDTree(refer_path)

    def LQR_Control(self, l_bias, d_l_bias, h_bias, d_h_bias):
        X = np.array([[l_bias], [d_l_bias], [h_bias], [d_h_bias]])
        xvhao = int(self.v * 100)
        if xvhao >= 3000:
            k = self.K[2999, :]
        else:
            k = self.K[xvhao, :]
        self.u_feedback = -np.dot(k, X)
        self.u_feedback_list.append(self.u_feedback)
        self.u_feedforward = self.tar_klist[self.i] * (self.L - self.lr * k[2] - (self.m * self.vx * self.vx / self.L)
                                                   * (self.lr / self.Cf + self.lf * k[2] / self.Cr - self.lf / self.Cr))
        self.u_feedforward_list.append(self.u_feedforward)
        self.u = - self.u_feedback + self.u_feedforward
        return self.u

    def track(self, car_model):
        # 找匹配点
        self.close_point(car_model)
        if self.i < len(self.tar_xlist)-1:
            # 匹配状态量
            self.match_x = self.tar_xlist[self.i]
            self.match_y = self.tar_ylist[self.i]
            self.match_h = self.tar_hlist[self.i]
            self.match_v = self.tar_vlist[self.i]
            self.match_k = self.tar_klist[self.i]

            # 自车状态量更新
            self.x = car_model.x
            self.y = car_model.y
            self.v = car_model.vx
            self.phi = car_model.phi

            if len(self.norepeat_phi_list) <= 1:
                d_phi = 0.001
            else:
                d_phi = (self.NormalizeAngle(self.phi - self.norepeat_phi_list[-2])) / (self.timestamp - self.norepeat_t_list[-2])

            match_n = np.array([[-math.sin(self.match_h)], [math.cos(self.match_h)]])  # 匹配点法向向量  math库中的三角函数运算符输入是弧度值
            match_tao = np.array([[math.cos(self.match_h)], [math.sin(self.match_h)]])  # 匹配点切向向量
            x_minus_matchx = np.array([self.x - self.match_x, self.y - self.match_y])  # 匹配点与车辆实际位置向量

            ed = np.dot(x_minus_matchx, match_n)
            self.ed = ed[0]
            self.ed_list.append(self.ed)
            if self.ed >= 9999 or self.ed <= -9999:  # 如果横向误差过大，则直接停止循迹
                print("ed out of side")
                self.lqr_mode = 0

            es = np.dot(x_minus_matchx, match_tao)
            es = es[0]

            theta_r = self.NormalizeAngle(self.match_h + self.match_k * es)

            self.ephi = self.NormalizeAngle(self.phi - theta_r)
            self.e_phi_list.append(self.ephi)
            d_s = self.v * math.cos(self.NormalizeAngle(self.phi - theta_r)) / (1 - self.match_k * self.ed)
            self.vx = d_s

            self.d_ed = self.v * math.sin(self.NormalizeAngle(self.phi - theta_r))
            self.d_ed_list.append(self.d_ed)
            self.d_ephi = d_phi - self.match_k * d_s
            self.d_e_phi_list.append(self.d_ephi)
            pi_steer = 180 / PI * self.LQR_Control(self.ed, self.d_ed, self.ephi, self.d_ephi)  # 弧度转角度
            pi_steer = pi_steer[0, ]
            max_steer = 20
            if pi_steer > max_steer:
                pi_steer = max_steer
            else:
                if pi_steer < -max_steer:
                    pi_steer = -max_steer
            self.steer = - float(pi_steer) / max_steer


if __name__ == '__main__':
    lqr = LQR_controller()
    dy_car = Dy_Car(1412, -112600, -89500, 2, 1.0, 1.9, 2.9, 1536, 3.14, 0, 0, 0.01, -1058.5960008467866, -12017.00799999886, 0)
    ki_car = Ki_Car(-1058.5960008467866, -12017.00799999886, 3.14, 2.9, 1, 0.01)
    pid = PID(1, 0, 0, 0.001)

    data = np.loadtxt('target_list_2024-10-21_14_49_29.txt', delimiter=' ')
    planning_x_list = data[:, 0]
    planning_y_list = data[:, 1]
    planning_h_list = data[:, 2]
    planning_v_list = data[:, 3]

    lqr.update_trajectory(planning_x_list, planning_y_list, planning_h_list, planning_v_list)
    lqr.LQR_Init()

    for i in range(2000):
        print("match_point = ", lqr.i)
        print("cur_v = ", dy_car.vx)
        print("tar_v = ", planning_v_list[lqr.i])
        print("_________________________________________________")
        print("\n")

        lqr.track(dy_car)
        dy_car.vx += pid.pid_lon(dy_car.vx, planning_v_list[lqr.i])
        dy_car.update(lqr.steer)

    """
    可视化
    """
    # 创建图形
    fig, axs = plt.subplots(2, 4, figsize=(10, 8))

    # 实际轨迹与规划轨迹x,y,h数据准备
    line_segments = []
    line_segments_ref = []

    x = dy_car.x_list
    y = dy_car.y_list
    h = dy_car.phi_list
    v = dy_car.v_list
    steer = dy_car.steer_list

    print("len(x) = ", len(x))
    print("len(steer) = ", len(steer))

    # x = ki_car.x_list
    # y = ki_car.y_list
    # h = ki_car.phi_list
    # v = ki_car.v_list
    # steer = ki_car.delta_f_list

    x_ref = planning_x_list
    y_ref = planning_y_list
    h_ref = planning_h_list
    v_ref = planning_v_list

    ed = lqr.ed_list
    ephi = lqr.e_phi_list
    u_fankui = lqr.u_feedback_list
    u_qiankui = lqr.u_feedforward_list

    i = 0
    sz = len(x)
    while i < sz - 1:
        line_segments_i = ((x[i], y[i]), h[i], 0.4)
        line_segments.append(line_segments_i)
        i += 1

    i = 0
    sz_ref = len(x_ref)
    while i < sz_ref - 1:
        line_segments_i_ref = ((x_ref[i], y_ref[i]), h_ref[i], 0.4)
        line_segments_ref.append(line_segments_i_ref)
        i += 1

    # 绘制实际轨迹的航向，位置图
    i = 0
    for start_point, angle, length in line_segments:
        i += 1
        # 计算终点
        end_x = start_point[0] + length * np.cos(angle)
        end_y = start_point[1] + length * np.sin(angle)
        end_point = (end_x, end_y)

        # 绘制线段
        axs[0, 0].plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], marker=',', color='blue')

        # 绘制起点，使用红色
        if i == 1:
            axs[0, 0].plot(start_point[0], start_point[1], marker='o', color='red', markersize=8,
                           label='Start' if 'Start' not in plt.gca().get_legend_handles_labels()[1] else "")

    # 绘制参考轨迹的航向，位置图
    i = 0
    for start_point, angle, length in line_segments_ref:
        i += 1
        # 计算终点
        end_x = start_point[0] + length * np.cos(angle)
        end_y = start_point[1] + length * np.sin(angle)
        end_point = (end_x, end_y)
        # 绘制线段
        axs[0, 0].plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], marker=',', color='green')
        # 绘制起点，使用红色
        if i == 1:
            axs[0, 0].plot(start_point[0], start_point[1], marker='o', color='red', markersize=8,
                           label='Start' if 'Start' not in plt.gca().get_legend_handles_labels()[1] else "")

    # 绘制实际跟踪速度图
    i = 0
    for x_i in x:
        axs[0, 1].plot(x_i, v[i], marker='.', color='blue')
        i += 1

    # 绘制参考跟踪速度图
    i = 0
    for x_i in x_ref:
        axs[0, 1].plot(x_i, v_ref[i], marker='.', color='green')
        i += 1

    # 绘制前轮转角（控制量）
    i = 0
    for x_i in x:
        axs[1, 0].plot(x_i, steer[i], marker='.', color='green')
        i += 1

    # 绘制ed图
    i = 0
    for x_i in x:
        axs[0, 2].plot(x_i, ed[i], marker='.', color='red')
        i += 1

    # 绘制ephi图
    i = 0
    for x_i in x:
        axs[0, 3].plot(x_i, ephi[i], marker='.', color='red')
        i += 1

    # 绘制heading图
    i = 0
    for x_i in x:
        axs[1, 1].plot(x_i, h[i], marker='.', color='red')
        i += 1

    # 绘制u_qiankui图
    i = 0
    for x_i in x:
        axs[1, 2].plot(x_i, u_qiankui[i], marker='.', color='red')
        i += 1

    # 绘制u_fankui图
    i = 0
    for x_i in x:
        axs[1, 3].plot(x_i, u_fankui[i], marker='.', color='red')
        i += 1

    # 设置图形属性
    axs[0, 0].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[0, 0].set_ylim(min(min(y), min(y_ref)) - 5, max(max(y), max(y_ref)) + 5)
    axs[0, 0].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 0].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 0].grid()
    axs[0, 0].set_title("xy and heading")

    axs[0, 1].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[0, 1].set_ylim(min(min(v), min(v_ref)) - 1, max(max(v), max(v_ref)) + 1)
    axs[0, 1].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 1].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 1].grid()
    axs[0, 1].set_title("v")

    axs[1, 0].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[1, 0].set_ylim(min(steer) - 1, max(steer) + 1)
    axs[1, 0].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 0].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 0].grid()
    axs[1, 0].set_title("steer")

    axs[1, 1].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[1, 1].set_ylim(min(h) - 1, max(h) + 1)
    axs[1, 1].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 1].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 1].grid()
    axs[1, 1].set_title("heading")

    axs[0, 2].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[0, 2].set_ylim(min(ed) - 1, max(ed) + 1)
    axs[0, 2].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 2].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 2].grid()
    axs[0, 2].set_title("ed")

    axs[0, 3].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[0, 3].set_ylim(min(ephi) - 1, max(ephi) + 1)
    axs[0, 3].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 3].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[0, 3].grid()
    axs[0, 3].set_title("ephi")

    axs[1, 2].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[1, 2].set_ylim(min(u_qiankui) - 1, max(u_qiankui) + 1)
    axs[1, 2].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 2].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 2].grid()
    axs[1, 2].set_title("u_qiankui")

    axs[1, 3].set_xlim(min(min(x), min(x_ref)) - 1, max(max(x), max(x_ref)) + 1)
    axs[1, 3].set_ylim(min(u_fankui) - 1, max(u_fankui) + 1)
    axs[1, 3].axhline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 3].axvline(0, color='black', linewidth=0.5, ls='--')
    axs[1, 3].grid()
    axs[1, 3].set_title("u_fankui")
    # 显示图形
    plt.tight_layout()
    plt.show()
