import numpy as np
import math


def Normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < - math.pi:
        angle += 2 * math.pi
    return angle


class Dy_Car:
    def __init__(self, M, Cf, Cr, vx, a, b, L, Iz, phi_init, d_phi_init, vy_init, T, x0, y0, steer_init):
        self.m = M
        self.cf = Cf
        self.cr = Cr
        self.vx = vx
        self.a = a
        self.b = b
        self.L = L
        self.Iz = Iz
        self.damping_factor = 0.1
        self.phi = phi_init
        self.d_phi = d_phi_init
        self.vy = vy_init
        self.dt = T
        self.x = x0
        self.y = y0
        self.steer = steer_init
        self.x_list = []
        self.y_list = []
        self.phi_list = []
        self.steer_list = []
        self.v_list = []

    def update(self, delta_f):
        self.steer = delta_f
        self.steer_list.append(self.steer)
        d_vy = ((self.cf + self.cr) / (self.m * self.vx)) * self.vy + (
                    ((self.a * self.cf - self.b * self.cr) / (self.m * self.vx)) - self.vx) * self.d_phi - (
                           self.cf / self.m) * self.steer
        d_d_phi = ((self.a * self.cf - self.b * self.cr) / (self.Iz * self.vx)) * self.vy + (
                    (self.a * self.a * self.cf + self.b * self.b * self.cr) / (self.Iz * self.vx)) * self.d_phi - (
                              self.a * self.cr / self.Iz) * self.steer
        self.d_phi += d_d_phi * self.dt
        self.phi += self.d_phi * self.dt
        self.phi = Normalize_angle(self.phi)
        self.vy += d_vy * self.dt
        beta = math.atan2(self.vy, self.vx)  # 质心侧偏角
        theta = Normalize_angle(self.phi + beta)
        v_ground = self.vx / math.cos(beta)
        vx_ground = v_ground * math.cos(theta)
        vy_ground = v_ground * math.sin(theta)
        self.x = self.x + vx_ground * self.dt
        self.y = self.y + vy_ground * self.dt
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.phi_list.append(self.phi)
        self.v_list.append(self.vx)


def deg2rad(angle):
    return angle * math.pi / 180


def steer_generate(length, num, amplitude, control_list):
    x = np.linspace(0, length, num)
    for x_i in x:
        y = amplitude * np.sin(0.5 * x_i)
        control_list.append(y)
    return x, control_list
