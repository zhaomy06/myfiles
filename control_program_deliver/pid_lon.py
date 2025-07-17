class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.err_list = []
        self.pre_err = 0
        self.u_lon = 0

    def pid_lon(self, cur_v, tar_v):
        cur_err = cur_v - tar_v
        if len(self.err_list) < 10:
            self.err_list.append(cur_err)
        else:
            self.err_list.pop(0)
            self.err_list.append(cur_err)
        u_p = cur_err * self.kp
        u_i = sum(self.err_list) * self.ki
        u_d = (cur_err - self.pre_err) / self.dt * self.kd
        self.u_lon = -(u_p + u_i + u_d)
        return self.u_lon


if __name__ == '__main__':
    pid = PID(1, 0, 0, 0.01)

    tar_lon_v = 10
    cur_lon_v = 0.0

    for i in range(1000):
        cur_lon_v = cur_lon_v + pid.pid_lon(cur_lon_v, tar_lon_v)
        print("tar_lon_v = ", tar_lon_v)
        print("cur_lon_v = ", cur_lon_v)


