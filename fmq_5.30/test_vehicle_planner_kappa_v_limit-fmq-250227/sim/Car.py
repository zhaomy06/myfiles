import numpy as np
import math
import P





class Car:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct #档位

    def update(self, a, delta, direct):
        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * P.P.dt
        self.y += self.v * math.sin(self.yaw) * P.P.dt
        self.yaw += self.v / P.P.WB * math.tan(delta) * P.P.dt
        self.direct = direct
        self.v += self.direct * a * P.P.dt
        self.v = self.limit_speed(self.v)

    def UpdateWithoutControl(self,x,y,yaw,v):
        self.x=x
        self.y=y
        self.yaw=yaw
        self.v=v

    @staticmethod
    def limit_input_delta(delta):
        if delta >= P.P.steer_max:
            return P.P.steer_max

        if delta <= -P.P.steer_max:
            return -P.P.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= P.P.speed_max:
            return P.P.speed_max

        if v <= P.P.speed_min:
            return P.P.speed_min

        return v