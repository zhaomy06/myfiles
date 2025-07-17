import numpy as np


class Ki_Car:
    def __init__(self, x0, y0, phi0, L, v0, T):  # L:wheel base
        self.x = x0
        self.y = y0
        self.phi = phi0
        self.L = L
        self.vx = v0
        self.dt = T
        self.x_list = []
        self.y_list = []
        self.phi_list = []
        self.v_list = []
        self.delta_f_list = []

    def update(self, delta_f):
        dx = self.vx * np.cos(self.phi)
        dy = self.vx * np.sin(self.phi)
        d_theta = self.vx * np.tan(delta_f) / self.L
        self.x += dx * self.dt
        self.y += dy * self.dt
        self.phi += d_theta * self.dt

        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.phi_list.append(self.phi)
        self.v_list.append(self.vx)
        self.delta_f_list.append(delta_f)
