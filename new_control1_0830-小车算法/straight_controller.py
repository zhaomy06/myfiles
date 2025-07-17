#修改了转向值输出问题，在原基础上除以45完成归一化变量
#2022年2月23日 17：24修改 陈祺遥
#更改了点集数量，精简参考点数
#2022年2月23日  17:28修改 陈祺遥
#2022年3月2日，增加stanley跟踪算法，待测试
import sys
from cmath import pi
import math
import numpy as np
PI=math.pi
k  = 0.2   # 增益参数
Kp = 0.22   # 速度P控制器系数
dt = 0.1   # 时间间隔，单位：s
L  = 1.18   # 车辆轴距，单位：m
pren=20
Linearspace = 300             # 安全边界横向距离
Linearend = 22.5              # 最长制动距离，以0.5的制动强度
Linearspeed = 5            # 限定最高车速，单位m/s
LinearMaxthr = 0.7            # 限定最大油门深度
Linearv = 10.0                # 高速临界点
Ksteer = 1                  # 高速转角实际比例
Lineara = 4                   # 制动时最大加速度
"""
3s制动，最高车速15m/s，制动强度0.5，制动距离22.5m,制动减速度5m/s2

"""

class Straight_controller():
    def __init__(self):
        self.i = 0  # 运行到的点位
        self.x = 0
        self.y = 0
        self.head = 0
        self.v = 10
        self.ki = 0
        self.kd = 0
        self.kp = 0.22
        self.kSteer = 0.5
        self.runmode = 0
        self.nn = 0
        self.steer = 0
        self.thr = 0
        self.brake = 0
        self.Cf = -30000
        self.Cr = -30000
        self.m = 100
        self.lf = 1
        self.lr = 1
        self.Iz = 100
        self.A = np.array([[0, 1, 0, 0],
                           [0, 2 * (self.Cf + self.Cr) / (self.m * self.v), -2 * (self.Cf + self.Cr) / self.m,
                            2 * (self.lf * self.Cf - self.lr * self.Cr) / (self.m * self.v)],
                           [0, 0, 0, 1],
                           [0, 2 * (self.lf * self.Cf - self.lr * self.Cr) / (self.Iz * self.v),
                            -2 * (self.lf * self.Cf - self.lr * self.Cr) / self.Iz,
                            2 * (self.lf * self.lf * self.Cf + self.lr * self.lr * self.Cr) / (self.Iz * self.v)]])
        self.B = np.array([[0], [-2 * self.Cf / self.m], [0], [-2 * self.lf * self.Cf / self.Iz]])
        self.Q = np.array([[4, 0, 0, 0],
                           [0, 4, 0, 0],
                           [0, 0, 4, 0],
                           [0, 0, 0, 4]])
        self.R = 100
        self.A_T = self.A.T
        self.B_T = self.B.T
        self.K = np.zeros((200, 4))
        self.y_bias_past = 0
        self.h_bias_past = 0
    def update_PID(self,ki,kp,kd,ksteer):
        self.ki=ki
        self.kd=kd
        self.kp=kp
        self.kSteer=ksteer

    def update_predata(self,xlist,ylist,hlist,vlist):
        self.LQR_Init()#放在init函数里加载不了K，所以放这了，竟然就能加载了，我也不知道为什么
        self.lastx = np.array(xlist)  # 当前对应数据点
        self.lasty = np.array(ylist)  #
        self.lastv = np.array(vlist)
        self.lasth = np.array(hlist)
        self.nextxlist, self.nextylist, self.nexthlist,self.nextvlist= self.genPre(self.lastx, self.lasty, self.lasth, self.lastv)
        self.len = len(self.nextxlist)
    def update_data(self,curx,cury,curhead,curv):
        self.x=curx
        self.y=cury
        self.head = curhead
        self.v=curv
    def closepoint(self):#pren为前瞻步数
        if self.i+pren>=len(self.nextxlist)-1:
            n=self.len-self.i
        else:
            n=pren
        self.nn=n
        bx=self.nextxlist[self.i:self.i+n]
        by=self.nextylist[self.i:self.i+n]
        x0list=np.ones(len(bx))*self.x
        y0list=np.ones(len(by))*self.y
        dMat=np.square(bx-x0list)+np.square(by-y0list)
        #print(dMat)
        #print(bx)
        self.i=np.where(dMat==np.min(dMat))[0][0]+self.i
        print('临近点下标'+str(self.i))

    def stanleytrack(self):
        self.closepoint()#self.i是最近点下标
        if self.i<self.len-5:
            prex=self.nextxlist[self.i]
            prey=self.nextylist[self.i]
            prex2=self.nextxlist[self.i+1]
            prey2=self.nextylist[self.i+1]
            prex3=self.nextxlist[self.i+2]
            prey3=self.nextylist[self.i+2]
            x=self.x
            y=self.y
            head=self.head/180*PI
            h1=self.headCal2(prex,prey,prex2,prey2)
            h2=self.headCal2(prex2,prey2,prex3,prey3)
            h=(h1+h2)/2/180*PI
            print('self.head'+str(self.head)+'head:'+str(head)+'preh'+str(h))
            tarv=self.tarV[self.i+1]
    # 计算横向误差
            if ((x - prex) * h - (y - prey)) > 0:
                error = abs(math.sqrt((x - prex) ** 2 + (y - prey) ** 2))
            else:
                error = -abs(math.sqrt((x - prex) ** 2 + (y - prey) ** 2))
            delta = (h - head + math.atan2(k * error, self.v)/10)
            print('delta:'+str(delta))

    #  限制车轮转角 [-30, 30]
            if delta > np.pi / 4.0:
                delta = np.pi / 4.0
            elif delta < - np.pi / 4.0:
                delta = - np.pi / 4.0
            self.steer=delta/(np.pi / 4.0)
    #p控制器控制油门
            if tarv-self.v>-2:
                self.thr = self.kp * (tarv - self.v)+self.ki
                self.brake=0
            else:
                if tarv-self.v<-4:
                    self.thr=0
                    self.brake=0.3

        else:
            print('track finish,stop!')
            self.steer=0
            self.thr=0
            self.brake=0.7

    def track(self):
        self.closepoint()
        #print(str(self.nn))

        if self.i < self.len - 5 and (self.nextxlist[self.i] != self.nextxlist[self.i + 2] or self.nextylist[self.i] != self.nextylist[self.i + 2]):
            match_x = self.nextxlist[self.i]
            match_y = self.nextylist[self.i]
            match_v = self.tarV[self.i]
            match_h = self.nexthlist[self.i]/180*math.pi#化为弧度值
            tarv = match_v
            head_hudu= self.head / 180 * math.pi#化为弧度值
            # 计算横向偏差，角度偏差，横向变化率，角度变化率。投影点heading近似认为是匹配点heading
            # 计算横向偏差
            match_tao = np.array([[-math.sin(match_h)], [math.cos(match_h)]])#math库中的三角函数运算符输入是弧度值
            d_x_matchx = np.array([self.x - match_x, self.y - match_y])
            l_bias = np.dot(d_x_matchx, match_tao)  # 车左偏为正，右偏为负
            l_bias = l_bias[0]
            #计算heading偏差
            if (head_hudu-match_h)>math.pi:
                h_bias=(head_hudu-match_h)-2*math.pi
            elif (head_hudu-match_h)<-math.pi:
                h_bias = (head_hudu - match_h) + 2 * math.pi
            else:
                h_bias = head_hudu - match_h
            #计算横向偏差变化率
            d_l_bias = self.v * (math.sin(h_bias))
            #计算heading偏差变化率
            d_h_bias = (h_bias - self.h_bias_past) / 0.1  # 这个0.1是track执行的周期，我还不知道具体是多少
            self.h_bias_past = h_bias  # 用作下一帧求d_h_bias
            #召唤lqr控制函数
            psteer = 180 / PI * self.LQR_Control(l_bias, d_l_bias, h_bias, d_h_bias)
            print(f'pssteer是{psteer}')
            psteer = - psteer[0, ]
            if psteer > 45:  # 转向限制
                psteer = 45
            else:
                if psteer < -45:
                    psteer = -45

            if tarv - self.v > -2:
                self.runmode = 0
            else:
                if tarv - self.v < -4:
                    self.runmode = 1
            '''
            if self.distCal(self.x,prex,self.y,prey)<0.2:
                self.runmode=1
            #暂时弃用
            '''
            if self.runmode == 0:
                thr = self.ki + self.kp * tarv
                # thr = self.kp * (tarv - self.v)+self.ki
                brake = 0.0
                print('加速' + str(thr))
            else:
                brake = 0.3
                thr = 0
                print('减速' + str(brake))
            # print ('转向='+str(psteer))
            # self.steer=1-math.cos((psteer/45)*pi/2)
            #self.steer = psteer / 45
            self.steer = float(psteer)/45
            self.thr = thr
            self.brake = brake
            print(f"thr={self.thr},brake={self.brake},steer={self.steer}")
            print(type(self.thr), type(self.brake), type(self.steer))
        else:
            print('track finish,stop!')
            self.steer = 0
            self.thr = 0
            self.brake = 0.7

    def LQR_Init(self):
        self.K = np.loadtxt('K.txt')
        '''max_cishu=500
        eps=0.01
        #下面代码计算的是速度从1到20，步长为0.1的区间内对应的矩阵K
        for i in range(1,200):
            P=self.Q
            self.A[1, 1] = 2 * (self.Cf + self.Cr) / (self.m * 0.1 * i)
            self.A[1, 3] = 2 * (self.lf * self.Cf - self.lr * self.Cr) / (self.m * 0.1 * i)
            self.A[3, 1] = 2 * (self.lf * self.Cf - self.lr * self.Cr) / (self.Iz * 0.1 * i)
            self.A[3, 3] = 2 * (self.lf * self.lf * self.Cf + self.lr * self.lr * self.Cr) / (self.Iz * 0.1 * i)
            self.A_T = self.A.T
            I=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            for j in range(max_cishu):
                AT_P_B=np.dot(np.dot(self.A_T,P),self.B)
                R_BT_P_B=np.linalg.inv(self.R+np.dot(np.dot(self.B_T,P),self.B))
                BT_P_A=np.dot(np.dot(self.B_T,P),self.A)
                P_diedai=np.dot(np.dot(self.A_T,P),self.A)-np.dot(np.dot(AT_P_B,R_BT_P_B),BT_P_A)+self.Q
                if(abs(P_diedai-P).max()<eps):
                    P=P_diedai
                    break
                else:
                    P=P_diedai
            K_temp=np.zeros((1,4))
            R_BT_P_B = np.linalg.inv(self.R + np.dot(np.dot(self.B_T, P), self.B))
            BT_P_A = np.dot(np.dot(self.B_T, P), self.A)
            K_temp=np.dot(R_BT_P_B,BT_P_A)
            self.K[i-1,:]=K_temp
        for i in range(0,200):
            if(self.K[i,0]==np.nan):
                self.K[i,:]=self.K[4,:]'''

    def LQR_Control(self, l_bias, d_l_bias, h_bias, d_h_bias):
        X = np.array([[l_bias], [d_l_bias], [h_bias], [d_h_bias]])
        print(l_bias)
        print(d_l_bias)
        print(h_bias)
        print(d_h_bias)
        xvhao=(int)(self.v*100)
        if(xvhao>=3000):
            k=self.K[2999,:]
        else:
            k = self.K[xvhao, :]
        print(f'用到的K值为{k}')
        u = -np.dot(k, X)
        print(f'u={u},k={k},x={X}')
        return u




    def steerCal(self,curX,curY,headDiff,preX,preY,kSteer):
        stc=kSteer*math.atan(1*math.sin(headDiff)/(self.distCal(curX,preX,curY,preY)+0.000001))
        return stc
    def distCal(self,x0,x1,y0,y1):
        dsc=pow((pow((x0-x1),2)+pow((y0-y1),2)),0.5)
        return dsc

    def headCal(self,curHead,curX,curY,preX,preY):
        tarHead=-math.atan2(preY-curY,preX-curX+0.0001)+PI/2
        if tarHead<=0:
            tarHead=tarHead+2*PI
        headDiff=tarHead-curHead
        if headDiff>PI:
            headDiff=headDiff-2*PI
        else:
             if headDiff<=-PI:
                 headDiff=headDiff+2*PI
        return headDiff

    def genPre(self,x ,y,h,v):
        # 预瞄数据生成
        xtemp=[x[0]]
        ytemp=[y[0]]
        htemp=[h[0]]
        vtemp=[v[0]]
        j=1
        k=0
        for i in  range(len(x)):#生成一系列有规律间隔的点
            if math.hypot(x[k]-x[i],y[k]-y[i])>0.5 and math.hypot(x[k]-x[i],y[k]-y[i])<20:
                xtemp.append(x[i])
                ytemp.append(y[i])
                htemp.append(h[i])
                vtemp.append(v[i])
                k=i
                j+=1#维数

        # x0=np.array(xtemp)
        # y0=np.array(ytemp)
        # h0=np.array(htemp)
        # v0=np.array(vtemp)
        # #h=np.zeros(j)
        # curv=np.zeros(j)
        # #for i in range(j-1):
        # #    h[i+1]=self.headCal2(x0[i],y0[i],x0[i+1],y0[i+1])
        # #h[0]=h[1]
        # #h=self.mah(h)
        # for i in range(j-1):
        #     curv[i+1]=self.dHeadCal(h[i],h[i+1])*math.pi/(180*math.hypot(x[i]-x[i+1],y[i]-y[i+1])+0.000001)
        # curv[0]=curv[1]
        # curv=self.ma(curv)
        # curv=self.ma(curv)
        # v0=self.ma(v0)
        # preD=np.zeros(j)
        # for i in range(j):
        #     preD[i]=0.6*(3+0.5*(20*v0[i]*3.6/80-80*abs(curv[i]/0.1)**2))
        #     if preD[i]<2:
        #         preD[i]=2
        #     elif preD[i]>15:
        #         preD[i]=15
        # preX=x0[j-1]*np.ones(j)
        # preY=y0[j-1]*np.ones(j)
        # preH=h0[j-1]*np.ones(j)
        # for i in range(j):
        #     for k in range(i,j):
        #         if math.hypot(x0[i]-x0[k],y0[i]-y0[k])>preD[i]:
        #             preX[i]=x0[k]
        #             preY[i]=y0[k]
        #             preH[i]=h0[k]
        #             break
        # return preX,preY,preH
        return xtemp,ytemp,htemp,vtemp

    def dHeadCal(self,h0,h1):
        # 角度差计算
        dH=h1-h0
        if dH>180:
            dH -=360
        elif dH<-180:
            dH += 360
        return dH
    def headCal2(self,x0,y0,x1,y1):
        # 角度计算
        h=np.arctan2(y1-y0,x1-x0)-math.pi/4
        if h<=0:
            h=h+2*math.pi
        return h*180/math.pi
    def mah(self,head0):
        # 角度均值滤波
        head1=head0
        n=head0.size
        for i in range(1,n):
            if i<3:
                head1[i]=self.angle_avg(head0[0:(2*i+1)])
            elif i>n-5:
                head1[i]=self.angle_avg(head0[(n-2*(n-i)+1):n])
            else:
                head1[i]=self.angle_avg(head0[i-3:i+4])
        return head1

    def ma(self,path0):
        # 普通均值滤波
        path1=path0
        n=path0.size
        for i in range(1,n):
            if i<6:
                path1[i]=np.mean(path0[0:(2*i+1)])#mean()函数功能：求取均值，此处为压缩列求平均值
            elif i>n-8:
                path1[i]=np.mean(path0[(n-2*(n-i)+1):n])
            else:
                path1[i]=np.mean(path0[i-6:i+7])
        return path1

    def angle_avg(self,angle):
        # 角度平均值计算
        last=angle[0]
        Sum=angle[0]
        n=angle.size
        for i in range(1,n):
            diff = self.dHeadCal(angle[i-1],angle[i])
            last += diff
            Sum += last
        aa=Sum/n
        if aa<=0:
            aa+=360
        return aa


    Linearx1 = 0.0              # 起点
    Linearx2 = 0.0              # 终点
    Lineary1 = 0.0
    Lineary2 = 0.0
    Linear_H = 0.0              # 起始航向角

    Linearz = 0.0               # 为了减少计算次数，提前计算 A2+B2+C2
    Lineararc = 0.0             # 直线角度
    Linear_k = 0.0
    Linear_b = 0.0

    end_dis = 1                 # 终止线距离终点位置
    end_line_k = 0.0
    end_line_b = 0.0
    end_begin = 0               # 判断点在直线上方或下方

    i = 0
    goal_v = 5.0
    bef_I_diff = 0.0
    I_diff = 0.0
    sum_diff = 0.0
    # diff_steer = 0.0
    bef_x = 0.0
    bef_y = 0.0
    bef_steer = 0.0
    model = 2       # 循迹模式规定
    # 新增车辆预测模块

    def predict_side(self):
        self.x = self.x + self.v * dt * math.sin(self.head*pi/180)
        self.y = self.y + self.v * dt * math.cos(self.head*pi/180)

    def xylinear(self, x1, y1, x2, y2):
        self.Linearz = math.sqrt((x1-x2)**2+(y1-y2)**2)             # 提前计算量
        self.Lineararc = np.arctan2(y2-y1, x2-x1)*180/pi            # 其范围为[-180,180]

        if self.Linearx1 == self.Linearx2:
            self.end_line_k = 0.0
            self.Linear_k = sys.maxsize
            self.Linear_b = - self.Linearx1
            if self.Lineary2 < self.Lineary1:
                self.end_line_b = self.Lineary2 + self.end_dis
            else:
                self.end_line_b = self.Lineary2 - self.end_dis
        elif self.Lineary2 == self.Lineary1:
            self.Linear_k = 0
            self.Linear_b = self.Lineary2
            self.end_line_k = sys.maxsize
            if self.Linearx2 > self.Linearx1:
                self.end_line_b = - self.Linearx2 + self.end_dis
            else:
                self.end_line_b = - self.Linearx2 - self.end_dis
        else:
            self.end_line_k = -1 / math.tan(self.Lineararc * pi/180)
            self.Linear_k = math.tan(self.Lineararc * pi/180)
            y = self.Lineary2 - self.end_dis * math.sin(self.Lineararc * pi / 180)
            x = self.Linearx2 - self.end_dis * math.cos(self.Lineararc * pi / 180)
            self.end_line_b = y - self.Linearx2 * x
        if self.Lineary1 < self.Linearx1 * self.end_line_k + self.end_line_b:
            self.end_begin = -1         # 在直线下方
        else:
            self.end_begin = 1          # 在直线上方
        if self.Linear_H > 180:
            self.Linear_H = self.Linear_H - 360

    def lin_judge(self, x1, y1):
        # 车到直线距离
        d = abs((self.Linearx1-x1)*(self.Lineary2-y1)-(self.Linearx2-x1)*(self.Lineary1-y1))/self.Linearz
        return d

    def end_line(self, x, y):       # 判断点是否到达终止线，若到达，则返回True
        if self.end_begin < 0:
            if y < x * self.end_line_k + self.end_line_b:
                return False
            else:
                return True
        else:
            if y > x * self.end_line_k + self.end_line_b:
                return False
            else:
                return True

    def linear(self):                                                # 直线算法
        # self.predict_side()             # 更新自己预测位置
        d = self.lin_judge(self.x, self.y)
        if d < Linearspace:
            # 加减速控制
            if self.distCal(self.x, self.Linearx2, self.y, self.Lineary2) < 1 or self.end_line(self.x, self.y):     # 判断是否到达目标点
                self.thr = 0.0
                self.steer = 0.0
                self.brake = 0.7
                print("The car has arrived at the designated location")
                return
            else:
                # 判断是否需要加速
                if self.v**2/2/Lineara-self.distCal(self.x, self.Linearx2, self.y, self.Lineary2) < -1:
                    if abs(self.v - self.goal_v) > 0.5:
                        if self.goal_v - self.v > 3:
                            self.thr = self.v * 0.1/3 + 0.1
                        else:
                            self.thr = self.v * 0.1/3 + (self.goal_v - self.v) * 0.1 / 3
                        self.brake = 0.0
                    elif abs(self.v-self.goal_v) < 0.5:
                        print(f"the car is cruising,and the cruising speed is {self.v},the throttle is {self.thr}")
                # 开始制动
                else:
                    self.thr = 0
                    self.brake = 0.2    # 匀速制动
        else:
            print('Beyond the security boundary! Stop!')
            self.steer = 0.0
            self.thr = 0.0
            self.brake = 0.7


    # 转向控制
        # 逻辑控制，若整车行驶与终点位置相背，以右转为规定方向，过大偏转角以哪个角度小往哪边转为原则
        # 左为负，右为正

        # 将航向角[0,360] 转化为[-180,180]
        self.Lineararc = np.arctan2(self.Lineary2 - self.y, self.Linearx2 - self.x) * 180 / pi  # 其范围为[-180,180]
        if self.Lineararc < 0:
            self.Lineararc = 360+self.Lineararc
        cursteer = 0
        # if model == 1:
        cursteer = self.head
        # if self.head > 180:
        #     cursteer = self.head - 360
        # else:
        #     cursteer = self.head
        # elif model == 2:
        #     if self.bef_x == 0 and self.bef_y == 0:
        #         cursteer = self.Lineararc
        #         self.bef_x = self.x
        #         self.bef_y = self.y
        #     else:
        #         cursteer = np.arctan2(self.y - self.bef_y, self.x - self.bef_x)*180/math.pi
        #         self.bef_x = self.x
        #         self.bef_y = self.y
        # 判断与预计角度偏离是否超过目标角
        # diff = math.asin(car_R/self.distCal(self.x, self.Linearx2, self.y, self.Lineary2))*180/math.pi*1.5 + 10
        # if abs(cursteer-self.Lineararc) > 35:
        #     if 180 > cursteer-self.Lineararc > 0 or cursteer-self.Lineararc < -180:
        #         self.steer = 0.90
        #         steer = 0.9*45
        #     else:
        #         self.steer = -0.90
        #         steer = -0.9*45
        #     self.thr = 0.05
        #     self.brake = 0.0

        if self.model == 1:        # 规定航向角模式
            diff_0 = self.Linear_H - cursteer
            if abs(diff_0) < 1:
                diff_0 = 0
            if self.y < self.Linear_k * self.x + self.Linear_b:
                stt = math.atan2(k * d, self.v)/10
            else:
                stt = - math.atan2(k * d, self.v)/10
            diff_0 = diff_0 + stt
        elif self.model == 2:    # 规定起点终点模式
            diff_0 = self.Lineararc - cursteer
            if abs(diff_0) < 1:
                diff_0 = 0

        # 理想二自由度模型转角,并且引入微分D项，消除波动误差
        diff_ = 0.1
        self.bef_I_diff = self.I_diff           # 记录上次转向角度
        self.I_diff = cursteer - self.Lineararc         # 计算本次转向角度
        if self.bef_I_diff > 0 and self.I_diff > 0:
            if self.I_diff - self.bef_I_diff > diff_:
                D_diff = self.I_diff - self.bef_I_diff      # 本次转向与上次转向误差
                self.sum_diff += D_diff                     # 累计误差
            else:
                D_diff = 0.0
                self.sum_diff = 0.0
        elif self.bef_I_diff < 0 and self.I_diff < -diff_:
            if self.I_diff - self.bef_I_diff < 0:
                D_diff = self.I_diff - self.bef_I_diff
                self.sum_diff += D_diff
            else:
                D_diff = 0.0
                self.sum_diff = 0.0
        else:
            D_diff = 0.0
            self.sum_diff = 0.0

        steer = math.atan(math.tan((self.I_diff + self.sum_diff * 0.5)*math.pi/180)*2) * 180 / math.pi     # 阿克曼转向角
        print(f"steer = {steer}")

        if abs(self.bef_steer - steer) > 0.5:
            if (self.bef_steer > 0 and steer < 0) or (self.bef_steer < 0 and steer > 0):
                if steer > 0:
                    steer = 1
                else:
                    steer = -1
            else:
                if self.bef_steer > 0 and steer > 0:
                    if steer > self.bef_steer:
                        steer = self.bef_steer + 0.5
                    else:
                        steer = steer
                else:
                    if steer < self.bef_steer:
                        steer = self.bef_steer - 0.5
                    else:
                        steer = steer
            self.bef_steer = steer




        self.steer = steer * self.kSteer / 45
        # 增加高速简单转向关系
        if self.v > Linearv:
            self.steer = self.steer * Ksteer

        self.i += 1
        if self.thr >= LinearMaxthr:   # 限制油门开度
            self.thr = LinearMaxthr - 0.01
