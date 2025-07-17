import numpy as np
import matplotlib.pyplot as plt


class Visual:
    def __init__(self, model, path, control_method):
        self.model = model
        self.path = path
        self.control_method = control_method

    def print_result(self):
        fig, axs = plt.subplots(2, 4, figsize=(10, 8))

        # 实际轨迹与规划轨迹x,y,h数据准备
        line_segments = []
        line_segments_ref = []

        x = self.model.x_list
        y = self.model.y_list
        h = self.model.phi_list
        v = self.model.v_list
        steer = self.model.steer_list

        x_ref = self.path.planning_x_list
        y_ref = self.path.planning_y_list
        h_ref = self.path.planning_h_list
        v_ref = self.path.planning_v_list

        ed = self.control_method.ed_list
        ephi = self.control_method.e_phi_list
        u_fankui = self.control_method.u_feedback_list
        u_qiankui = self.control_method.u_feedforward_list

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
