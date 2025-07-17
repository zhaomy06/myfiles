import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Wedge
from matplotlib.lines import Line2D
import math

# 之前定义的代码，请确保已有的库和常量定义都要包含在内

class CarPatch:
    def __init__(self, x, y, yaw, steer, C, color='black'):
        self.C=C
        wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                          [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])
        
        rl_wheel=wheel.copy()
        rr_wheel=wheel.copy()
        fl_wheel=wheel.copy()
        fr_wheel=wheel.copy()

        rlWheel = self.rotate_for_rlwheel(rl_wheel, yaw, x, y )
        rrWheel = self.rotate_for_rrwheel(rr_wheel, yaw, x, y )

        flWheel = self.rotate_for_flwheel(fl_wheel, steer,yaw, x, y )
        frWheel = self.rotate_for_frwheel(fr_wheel, steer,yaw, x ,y )
        # flWheel = self.rotate_and_translate(wheel, 0, x + C.WB, y + C.WD / 2)
        # frWheel = self.rotate_and_translate(wheel, 0, x + C.WB, y - C.WD / 2)

        car_body = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                             [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])
        body = self.rotate_and_translate(car_body, yaw, x, y)

        self.car_poly = Polygon(body.T, True, fill=False, color=color)
        self.fl_wheel_poly = Polygon(flWheel.T, True, fill=False, color=color)
        self.fr_wheel_poly = Polygon(frWheel.T, True, fill=False, color=color)
        self.rl_wheel_poly = Polygon(rlWheel.T, True, fill=False, color=color)
        self.rr_wheel_poly = Polygon(rrWheel.T, True, fill=False, color=color)
        self.arrow = Arrow(x, y, yaw, C.WB * 0.6, color)

    def rotate_and_translate(self, points, yaw, x, y):
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        return np.dot(Rot1, points) + np.array([[x], [y]])

    def rotate_for_flwheel(self, points, steer, yaw, x, y):
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        Rot2 = np.array([[np.cos(steer), np.sin(steer)],
                    [-np.sin(steer), np.cos(steer)]])
        fl_wheel=np.dot(Rot2,points)
        fl_wheel += np.array([[self.C.WB], [self.C.WD / 2]])
        fl_wheel = np.dot(Rot1, fl_wheel)
        fl_wheel += np.array([[x], [y]])

        return fl_wheel
    
    def rotate_for_frwheel(self, points, steer, yaw, x, y):
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        Rot2 = np.array([[np.cos(steer), np.sin(steer)],
                    [-np.sin(steer), np.cos(steer)]])
        fr_wheel=np.dot(Rot2,points)
        fr_wheel += np.array([[self.C.WB], [-self.C.WD / 2]])
        fr_wheel = np.dot(Rot1, fr_wheel)
        fr_wheel += np.array([[x], [y]])

        return fr_wheel
    
    def rotate_for_rlwheel(self, points, yaw, x, y):
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        points[1, :] += self.C.WD / 2
        rl_wheel=np.dot(Rot1,points)
        rl_wheel += np.array([[x], [y]])
        return rl_wheel
    
    def rotate_for_rrwheel(self, points, yaw, x, y):
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        points[1, :] -= self.C.WD  / 2
        rr_wheel=np.dot(Rot1,points)
        rr_wheel += np.array([[x], [y]])
        return rr_wheel

    def add_to_axes(self, ax):
        ax.add_patch(self.car_poly)
        ax.add_patch(self.fl_wheel_poly)
        ax.add_patch(self.fr_wheel_poly)
        ax.add_patch(self.rl_wheel_poly)
        ax.add_patch(self.rr_wheel_poly)
        # 如果需要绘制箭头，可以取消注释下面的代码并进一步实现箭头的绘制方法
        # self.arrow.add_to_axes(ax)

class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.4 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + np.pi - angle
        theta_hat_R = theta + np.pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        self.main_line = Line2D([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
        self.left_line = Line2D([x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L], color=c, linewidth=w)
        self.right_line = Line2D([x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R], color=c, linewidth=w)

    def add_to_axes(self, ax):
        ax.add_line(self.main_line)
        ax.add_line(self.left_line)
        ax.add_line(self.right_line)