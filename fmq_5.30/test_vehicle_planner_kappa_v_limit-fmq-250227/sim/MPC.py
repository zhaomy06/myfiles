import cvxpy
import numpy as np
import Car
import P
import math


class PATH:
    def __init__(self, cx, cy, cyaw, ck):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.length = len(cx)
        self.ind_old = 0

    def nearest_index(self, node):
        """
        calc index of the nearest node in N steps
        :param node: current information
        :return: nearest index, lateral distance to ref point
        """

        dx = [node.x - x for x in self.cx[self.ind_old: (self.ind_old + P.P.N_IND)]]
        dy = [node.y - y for y in self.cy[self.ind_old: (self.ind_old + P.P.N_IND)]]
        dist = np.hypot(dx, dy)

        ind_in_N = int(np.argmin(dist))
        ind = self.ind_old + ind_in_N
        self.ind_old = ind
        # 老王的那种计算横向偏差的方法
        rear_axle_vec_rot_90 = np.array([[math.cos(node.yaw + math.pi / 2.0)],
                                         [math.sin(node.yaw + math.pi / 2.0)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        er = er[0][0]

        return ind, er


class MPC:
    def __init__(self, x, y, yaw, k, v):
        self.path = PATH(x, y, yaw, k)
        self.v = v
        self.delta_opt, self.a_opt = None, None

        self.Kp=1
        self.Ki=0.01
        self.Kd=0.01

    def update_trajectory(self, x,y,yaw,k,v):
        self.v = v
        self.path.cx=x
        self.path.cy=y
        self.path.cyaw=yaw
        self.path.ck=k
        self.path.length=len(x)
        self.path.ind_old=0

    def cal_k(self, x, y):
        k = [0]
        print("x", x)
        print("y", y)
        for i in range(1, len(x) - 1):
            a_index = i - 1
            b_index = i
            c_index = i + 1
            d_a = math.sqrt(pow(x[b_index] - x[c_index], 2) + pow(y[b_index] - y[c_index], 2))
            d_b = math.sqrt(pow(x[a_index] - x[c_index], 2) + pow(y[a_index] - y[c_index], 2))
            d_c = math.sqrt(pow(x[b_index] - x[a_index], 2) + pow(y[b_index] - y[a_index], 2))
            if d_a < 0.01:
                d_a = 0.01
            if d_c < 0.01:
                d_c = 0.01
            cos = (d_a * d_a + d_c * d_c - d_b * d_b) / (2 * d_a * d_c)
            # 这里可能是精度上有差异，竟然能算出大于1的cos
            if ((1 - cos * cos) < 0):
                k_temp = 0.01
            else:
                sin = math.sqrt(1 - cos * cos)
                k_temp = 2 * sin / d_b
            k.append(k_temp)
            # print("第",i+1,"个k=",k_temp)
        k.append(0)
        return k
    
    def CalPIDAcc(self,car):
        match_idx, _=self.path.nearest_index(car)

        tar_v=self.v[match_idx]
        err=car.v-tar_v

        P=-self.Kp*err

        I=0.0

        D=0.0

        return P+I+D



    def cal_control(self, car):
        z_ref, target_ind = self.calc_ref_trajectory_in_T_step(car, self.path, self.v)
        #print("看看源头的length", self.path.length, "len of x=", len(self.path.cx))
        #好的，这个离奇的path长度会变化的bug应该是因为有两辆车，main_car的线直接用的参考线，所以长
        z0 = [car.x, car.y, car.v, car.yaw]
        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = self.linear_mpc_control(z_ref, z0, self.a_opt, self.delta_opt)
        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]

        a_pid=self.CalPIDAcc(car)

        return delta_exc, a_exc, a_pid

    def linear_mpc_control(self, z_ref, z0, a_old, delta_old):
        """
        linear mpc controller
        :param z_ref: reference trajectory in T steps
        :param z0: initial state vector
        :param a_old: acceleration of T steps of last time
        :param delta_old: delta of T steps of last time
        :return: acceleration and delta strategy based on current information
        """

        if a_old is None or delta_old is None:
            a_old = [0.0] * P.P.T
            delta_old = [0.0] * P.P.T

        x, y, yaw, v = None, None, None, None

        for k in range(P.P.iter_max):
            z_bar = self.predict_states_in_T_step(z0, a_old, delta_old, z_ref)
            a_rec, delta_rec = a_old[:], delta_old[:]
            a_old, delta_old, x, y, yaw, v = self.solve_linear_mpc(z_ref, z_bar, z0, delta_old)

            du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
            du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

            if max(du_a_max, du_d_max) < P.P.du_res:
                break

        return a_old, delta_old, x, y, yaw, v

    def predict_states_in_T_step(self, z0, a, delta, z_ref):
        """
        given the current state, using the acceleration and delta strategy of last time,
        predict the states of vehicle in T steps.
        :param z0: initial state
        :param a: acceleration strategy of last time
        :param delta: delta strategy of last time
        :param z_ref: reference trajectory
        :return: predict states in T steps (z_bar, used for calc linear motion model)
        """

        z_bar = z_ref * 0.0

        for i in range(P.P.NX):
            z_bar[i, 0] = z0[i]

        node = Car.Car(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

        for ai, di, i in zip(a, delta, range(1, P.P.T + 1)):
            node.update(ai, di, 1.0)
            z_bar[0, i] = node.x
            z_bar[1, i] = node.y
            z_bar[2, i] = node.v
            z_bar[3, i] = node.yaw

        return z_bar

    def solve_linear_mpc(self, z_ref, z_bar, z0, d_bar):
        """
        solve the quadratic optimization problem using cvxpy, solver: OSQP
        :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
        :param z_bar: predicted states in T steps
        :param z0: initial state
        :param d_bar: delta_bar
        :return: optimal acceleration and steering strategy
        """

        z = cvxpy.Variable((P.P.NX, P.P.T + 1))
        u = cvxpy.Variable((P.P.NU, P.P.T))

        cost = 0.0
        constrains = []

        for t in range(P.P.T):
            cost += cvxpy.quad_form(u[:, t], P.P.R)
            cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], P.P.Q)

            A, B, C = self.calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])

            constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

            if t < P.P.T - 1:
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], P.P.Rd)
                constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= P.P.steer_change_max * P.P.dt]

        cost += cvxpy.quad_form(z_ref[:, P.P.T] - z[:, P.P.T], P.P.Qf)

        constrains += [z[:, 0] == z0]
        constrains += [z[2, :] <= P.P.speed_max]
        constrains += [z[2, :] >= P.P.speed_min]
        constrains += [cvxpy.abs(u[0, :]) <= P.P.acceleration_max]
        constrains += [cvxpy.abs(u[1, :]) <= P.P.steer_max]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)

        a, delta, x, y, yaw, v = None, None, None, None, None, None

        if prob.status == cvxpy.OPTIMAL or \
                prob.status == cvxpy.OPTIMAL_INACCURATE:
            x = z.value[0, :]
            y = z.value[1, :]
            v = z.value[2, :]
            yaw = z.value[3, :]
            a = u.value[0, :]
            delta = u.value[1, :]
        else:
            print("Cannot solve linear mpc!")

        return a, delta, x, y, yaw, v

    def calc_linear_discrete_model(self, v, phi, delta):
        """
        calc linear and discrete time dynamic model.
        :param v: speed: v_bar
        :param phi: angle of vehicle: phi_bar
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        A = np.array([[1.0, 0.0, P.P.dt * math.cos(phi), - P.P.dt * v * math.sin(phi)],
                      [0.0, 1.0, P.P.dt * math.sin(phi), P.P.dt * v * math.cos(phi)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, P.P.dt * math.tan(delta) / P.P.WB, 1.0]])

        B = np.array([[0.0, 0.0],
                      [0.0, 0.0],
                      [P.P.dt, 0.0],
                      [0.0, P.P.dt * v / (P.P.WB * math.cos(delta) ** 2)]])

        C = np.array([P.P.dt * v * math.sin(phi) * phi,
                      -P.P.dt * v * math.cos(phi) * phi,
                      0.0,
                      -P.P.dt * v * delta / (P.P.WB * math.cos(delta) ** 2)])

        return A, B, C

    def calc_ref_trajectory_in_T_step(self, node, ref_path, sp):
        """
        calc referent trajectory in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param node: current information
        :param ref_path: reference path: [x, y, yaw]
        :param sp: speed profile (designed speed strategy)
        :return: reference trajectory
        """

        z_ref = np.zeros((P.P.NX, P.P.T + 1))
        length = ref_path.length
        ind, _ = ref_path.nearest_index(node)

        z_ref[0, 0] = ref_path.cx[ind]
        z_ref[1, 0] = ref_path.cy[ind]
        z_ref[2, 0] = sp[ind]
        z_ref[3, 0] = ref_path.cyaw[ind]

        for i in range(1, P.P.T + 1):
            index=min(ind+i,length-1)

            z_ref[0, i] = ref_path.cx[index]
            z_ref[1, i] = ref_path.cy[index]
            z_ref[2, i] = sp[index]
            z_ref[3, i] = ref_path.cyaw[index]

        return z_ref, ind
