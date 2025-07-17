import math
import numpy as np

pren = 20
PI = math.pi
# 别忘了把i重置


class LQR_controller:
    def __init__(self):
        self.lf = 0.71
        self.lr = 1.2-0.71
        self.m = 125.76
        self.L = 1.2
        self.Cf = -2020
        self.Cr = -2020
        self.i = 0  # 运行到的点位(匹配点index)
        self.x = 0
        self.y = 0
        self.head = 0
        self.v = 10
        self.vx = 0  # 近似为d_s
        self.timestamp = 0
        # self.timestamp_past = 0
        self.phi = 0
        # self.phi_past = 0
        self.ki = 0
        self.kd = 0
        self.kp = 0.15
        self.steer = 0
        self.thr = 0
        self.brake = 0
        self.K = np.zeros((3000, 4))
        self.y_bias_past = 0
        self.h_bias_past = 0
        self.I_jilu = np.zeros(10)
        self.I_jilu = list(self.I_jilu)
        self.pre_e = 0
        self.tar_xlist = None
        self.tar_ylist = None
        self.tar_hlist = None
        self.tar_vlist = None
        self.tar_klist = None
        self.tar_tlist = None
        self.lqr_mode = 0    # 1是开始寻迹，0是结束寻迹
        # self.fist_run = True  # 处理第一帧时的 d_phi
        self.repeat_flag = False
        self.norepeat_phi_list = []
        self.norepeat_t_list = []
        self.t_stamp = 0.1  # 线性差值时间步长
        self.ed = 0
        self.d_ed = 0
        self.ephi = 0
        self.d_ephi = 0
        self.match_x = 0
        self.match_y = 0
        self.match_h = 0
        self.match_v = 0
        self.match_k = 0
        self.u_fankui = 0
        self.u_qiankui = 0
        self.u = 0
        self.log_list = [self.timestamp, self.x, self.y, self.phi, self.v, self.match_x, self.match_y, self.match_h,
                         self.match_v, self.match_k, 1.0 if self.repeat_flag else 0.0, self.i, self.ed, self.d_ed, self.ephi,
                         self.d_ephi, float(self.u_fankui), float(self.u_qiankui), float(self.u)]  # log

    def update_ego_state(self, curx, cury, curhead, curv, curtime, repeat_flag):
        self.x = curx
        self.y = cury
        self.head = curhead
        self.phi = self.head
        self.v = curv
        self.timestamp = curtime
        self.repeat_flag = repeat_flag
        if not self.repeat_flag:  # 如果未重复
            self.norepeat_t_list.append(self.timestamp)
            self.norepeat_phi_list.append(self.phi)


    def update_trajectory(self, xlist, ylist, hlist, vlist, tlist):
        self.LQR_Init()  # 初始化K
        xlist_temp, ylist_temp, hlist_temp, vlist_temp, tlist_temp = self.preprocess_data(xlist, ylist, hlist, vlist, tlist)
        klist_temp = self.cal_k(xlist_temp, ylist_temp)
        
        # self.tar_xlist, self.tar_ylist, self.tar_hlist, self.tar_vlist, self.tar_klist,self.tar_tlist = self.linear_interpolation(xlist_temp, ylist_temp, hlist_temp, vlist_temp, klist_temp,tlist_temp)
        self.tar_xlist = xlist_temp
        self.tar_ylist = ylist_temp
        self.tar_hlist = hlist_temp
        self.tar_vlist = vlist_temp
        self.tar_klist = klist_temp
        self.tar_tlist = tlist_temp

        # 算曲率要在数据预处理之后，要不会出现非常奇怪的曲率111111
        # print("self.tar_xlist=",self.tar_xlist)
        # print("self.tar_ylist=",self.tar_ylist)
        # print("self.tar_hlist=",self.tar_hlist)
        # print("self.tar_vlist=",self.tar_vlist)
        # print("self.tar_tlist=",self.tar_tlist)
        print("self.tar_klist=", self.tar_klist)
        for i in range(0, len(self.tar_vlist)):
            if self.tar_vlist[i] < 0.1:
                self.tar_vlist[i] = 0.1

    def preprocess_data(self, x, y, h, v, t):
        xtemp = [x[0]]
        ytemp = [y[0]]
        htemp = [h[0]]
        vtemp = [v[0]]
        ttemp = [t[0]]
        sum_d = 0
        for i in range(1, len(x)):  # 生成一系列有规律间隔的点
            sum_d += math.hypot(x[i] - x[i-1], y[i] - y[i-1])
            if sum_d > 0.1:
                xtemp.append(x[i])
                ytemp.append(y[i])
                htemp.append(h[i])
                vtemp.append(v[i])
                ttemp.append(t[i])
                sum_d = 0
        return xtemp, ytemp, htemp, vtemp, ttemp

    def linear_interpolation(self, x_temp, y_temp, h_temp, v_temp, k_temp, t_temp):  # 线性插值
        new_t_temp = np.arange(t_temp[0], t_temp[-1], self.t_stamp)  # self.t_stamp 未知
        print("new_t_temp[-1]=", new_t_temp[-1])
        print("t_temp=[-1]", t_temp[-1])
        new_x_temp = []
        new_y_temp = []
        new_h_temp = []
        new_v_temp = []
        new_k_temp = []
        for time in new_t_temp:
            match_idx = 99999999
            for i in range(len(t_temp)-1):
                if (time >= t_temp[i]) and (time <= t_temp[i+1]):
                    match_idx = i
                    break
            print("index=", match_idx)
            ratio = (time-t_temp[match_idx])/(t_temp[match_idx+1]-t_temp[match_idx])
            x_linear = x_temp[match_idx]+ratio*(x_temp[match_idx+1]-x_temp[match_idx])
            y_linear = y_temp[match_idx]+ratio*(y_temp[match_idx+1]-y_temp[match_idx])
            v_linear = v_temp[match_idx]+ratio*(v_temp[match_idx+1]-v_temp[match_idx])
            k_linear = k_temp[match_idx]+ratio*(k_temp[match_idx+1]-k_temp[match_idx])
            h_linear = self.NormalizeAngle(h_temp[match_idx]+ratio*self.NormalizeAngle(h_temp[match_idx+1]-h_temp[match_idx]))
            new_x_temp.append(x_linear)
            new_y_temp.append(y_linear)
            new_v_temp.append(v_linear)
            new_k_temp.append(k_linear)
            new_h_temp.append(h_linear)
            print("new_v_temp", new_v_temp)
        return new_x_temp, new_y_temp, new_h_temp, new_v_temp, new_k_temp, new_t_temp

    def closepoint(self):  # pren为前瞻步数
        if self.i + pren >= len(self.tar_xlist):
            n = len(self.tar_xlist)-self.i
        else:
            n = pren
        bx = self.tar_xlist[self.i:self.i+n]
        by = self.tar_ylist[self.i:self.i+n]
        x0list = np.ones(len(bx))*self.x
        y0list = np.ones(len(by))*self.y
        dMat = np.square(bx-x0list)+np.square(by-y0list)
        self.i = np.where(dMat == np.min(dMat))[0][0] + self.i

    def NormalizeAngle(self, angle):
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle < - math.pi:
            angle += 2 * math.pi
        return angle

    def track(self):
        self.closepoint()  # 找匹配点
        print("execute track")
        if (self.i < len(self.tar_xlist)-1) and self.lqr_mode == 1:
            self.match_x = self.tar_xlist[self.i]
            self.match_y = self.tar_ylist[self.i]
            self.match_h = self.tar_hlist[self.i]
            self.match_v = self.tar_vlist[self.i]
            self.match_k = self.tar_klist[self.i]
            # phi = self.clockwise_0_360_to_pi_negative_pi(self.head)

            # 计算横向偏差，航向偏差，横向偏差变化率，航向偏差变化率
            # 差分计算d_phi(当前车辆横摆角速度)
            if len(self.norepeat_phi_list) <= 1:  # 前两帧
                d_phi = 0.001
                # self.phi_past = self.phi
                # self.timestamp_past = self.timestamp
                # self.fist_run = False
            else:  # 从记录的未重复列表末尾位取差分值
                # if self.repeat_flag == True:
                d_phi = (self.phi - self.norepeat_phi_list[-2]) / (self.timestamp - self.norepeat_t_list[-2])
                print("d_phi=", d_phi)
                    # self.phi_past = self.phi
                    # self.timestamp_past = self.timestamp

            # 计算匹配点切向量法向量，匹配点与实际点位矢
            match_n = np.array([[-math.sin(self.match_h)], [math.cos(self.match_h)]])  # 匹配点法向向量  math库中的三角函数运算符输入是弧度值
            match_tao = np.array([[math.cos(self.match_h)], [math.sin(self.match_h)]])  # 匹配点切向向量
            x_minus_matchx = np.array([self.x - self.match_x, self.y - self.match_y])  # 匹配点与车辆实际位置向量

            # 计算横向偏差  车左偏为正，右偏为负
            ed = np.dot(x_minus_matchx, match_n)
            self.ed = ed[0]
            if self.ed >= 9999 or self.ed <= -9999:  # ################？
                self.lqr_mode = 0

            # 匹配点与投影点之间的弧长es
            es = np.dot(x_minus_matchx, match_tao)
            es = es[0]

            # 投影点航向theta_r
            theta_r = self.match_h + self.match_k * es

            # 计算航向偏差ephi
            if (self.phi - theta_r) > math.pi:
                self.ephi = (self.phi - theta_r) - 2 * math.pi
            elif (self.phi - theta_r) < -math.pi:
                self.ephi = (self.phi - theta_r) + 2 * math.pi
            else:
                self.ephi = self.phi - theta_r

            # 投影点在frenet坐标轴上位矢的导数s_dot
            d_s = self.v * math.cos(self.phi - theta_r) / (1 - self.match_k * self.ed)
            self.vx = d_s

            # 计算横向偏差变化率
            self.d_ed = self.v * math.sin(self.NormalizeAngle(self.phi - theta_r))

            # 计算航向偏差变化率d_ephi
            self.d_ephi = d_phi - self.match_k * d_s
            # 计算横向偏差，角度偏差，横向变化率，角度变化率。投影点heading近似认为是匹配点heading
            # 计算横向偏差
            # match_tao = np.array([[-math.sin(match_h)], [math.cos(match_h)]])  # math库中的三角函数运算符输入是弧度值
            # d_x_matchx = np.array([self.x - match_x, self.y - match_y])
            # l_bias = np.dot(d_x_matchx, match_tao)  # 车左偏为正，右偏为负
            # l_bias = l_bias[0]
            # if l_bias >= 9999 or l_bias <= -9999:
            #     self.lqr_mode = 0
            # # 计算heading偏差
            # # if (head_hudu-match_h)>math.pi:
            # #     h_bias=(head_hudu-match_h)-2*math.pi
            # # elif (head_hudu-match_h)<-math.pi:
            # #     h_bias = (head_hudu - match_h) + 2 * math.pi
            # # else:
            # #     h_bias = head_hudu - match_h
            # h_bias = self.NormalizeAngle(self.phi-match_h)
            # # 计算横向偏差变化率
            # d_l_bias = self.v * (math.sin(h_bias))
            # # 计算heading偏差变化率
            # d_h_bias = (h_bias - self.h_bias_past) / 0.1  # 这个0.1是track执行的周期，我还不知道具体是多少
            # self.h_bias_past = h_bias  # 用作下一帧求d_h_bias
            # 召唤lqr控制函数
            psteer = 180 / PI * self.LQR_Control(self.ed, self.d_ed, self.ephi, self.d_ephi)
            psteer = - psteer[0, ]
            max_steer = 20
            if psteer > max_steer:  # 转向限制
                psteer = max_steer
            else:
                if psteer < -max_steer:
                    psteer = -max_steer

            v_error = self.v-self.match_v
            a_zhengze = self.PID_control(v_error)
            print("v_error=",v_error)
            print("a_zhengze=",a_zhengze)
            if a_zhengze >= 0.5:
                a_zhengze = 0.5
            if a_zhengze <= -1:
                a_zhengze = -1
            if a_zhengze >= 0:
                thr = a_zhengze
                brake = 0
            else:
                thr = 0
                brake = -a_zhengze
            self.steer = float(psteer)/max_steer
            self.thr = thr
            self.brake = brake
        else:
            self.lqr_mode = 0
            self.steer = 0
            self.thr = 0
            self.brake = 0.95

        print("self.steer=",self.steer)
        print("self.thr=",self.thr)
        print("self.brake=",self.brake)

        # 停住之后初始化
        if self.lqr_mode == 0:
            self.steer = 0
            self.thr = 0
            self.brake = 0.8
            self.i = 0
            self.I_jilu = np.zeros(10)
            self.I_jilu = list(self.I_jilu)
            self.y_bias_past = 0
            self.h_bias_past = 0
            self.pre_e = 0
            self.timestamp = 0
            # self.timestamp_past = 0
            self.phi = 0
            # self.phi_past = 0
            # self.fist_run = True
        
        self.log_list = [self.timestamp, self.x, self.y, self.phi, self.v, self.match_x, self.match_y, self.match_h,
                         self.match_v, self.match_k, 1.0 if self.repeat_flag else 0.0, self.i, self.ed, self.d_ed, self.ephi,
                         self.d_ephi, float(self.u_fankui), float(self.u_qiankui), float(self.u)]

        return self.lqr_mode

    def LQR_Init(self):
        self.K = np.loadtxt('K.txt')

    def LQR_Control(self, l_bias, d_l_bias, h_bias, d_h_bias):
        X = np.array([[l_bias], [d_l_bias], [h_bias], [d_h_bias]])
        xvhao = (int)(self.v*100)
        if(xvhao >= 3000):
            k = self.K[2999, :]
        else:
            k = self.K[xvhao, :]
        print("k=",k)
        print("X=",X)            
        self.u_fankui = -np.dot(k, X)
        # u_qiankui = (self.m*self.v*self.v*self.tar_klist[self.i]/self.L)*(self.lr/(2*self.Cf)-self.lf/(2*self.Cr)
        #         +self.lf*k[2]/(2*self.Cr))\
        #         +self.L*self.tar_klist[self.i]\
        #         -self.lr*self.tar_klist[self.i]*k[2]
        self.u_qiankui = self.tar_klist[self.i]*(self.L-self.lr*k[2]-(self.m*self.vx*self.vx/self.L)
                                            *(self.lr/self.Cf+self.lf*k[2]/self.Cr-self.lf/self.Cr))
        # print("u反馈=", self.u_fankui)
        # print("u前馈=", self.u_qiankui)
        self.u = self.u_fankui + self.u_qiankui
        return self.u

    def PID_control(self, e):
        e = e
        self.I_jilu.pop(0)
        self.I_jilu.append(e)

        d_temp = (e-self.pre_e)/0.1
        self.pre_e = e

        thr_P = self.kp*e
        thr_I = self.ki*sum(self.I_jilu)
        thr_D = self.kd*d_temp

        u = thr_P+thr_I+thr_D
        u = -u
        return u

    def cal_k(self, x, y):
        k = [0]
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
        k.append(0)
        return k

    # def control_log(self):
    #     self.log_list.append(self.timestamp)
    #     self.log_list.append(self.x)
    #     self.log_list.append(self.y)
    #     self.log_list.append(self.phi)
    #     self.log_list.append(self.v)
    #     self.log_list.append(self.match_x)
    #     self.log_list.append(self.match_y)
    #     self.log_list.append(self.match_h)
    #     self.log_list.append(self.match_v)
    #     self.log_list.append(self.match_k)
    #     self.log_list.append(self.repeat_flag)
    #     self.log_list.append(self.i)
    #     self.log_list.append(self.ed)
    #     self.log_list.append(self.d_ed)
    #     self.log_list.append(self.ephi)
    #     self.log_list.append(self.d_ephi)
    #     self.log_list.append(self.u_fankui)
    #     self.log_list.append(self.u_qiankui)
    #     self.log_list.append(self.u)

