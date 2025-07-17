import math
import numpy as np
import matplotlib.pyplot as plt
import P
import Car
import Reference_line
import cvxpy
import os


class Planner:
    def __init__(self, ref_line, tar_ref_line, lane_width,car):
        self.reference_line = ref_line
        self.tar_reference_line = tar_ref_line
        self.lane_width=lane_width
        self.car = car
        self.CalChangeLaneSampleTable()
        self.k_max = 0.3
        self.s_dis = 0.2

    def CalChangeLaneSampleTable(self):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_name=current_directory+"/change_lane_sample.txt"
        print("file_name=",file_name)
        sample_data = np.loadtxt(file_name, delimiter=' ')
        x_sample = sample_data[:, 0]
        y_sample = sample_data[:, 1]
        theta_sample=sample_data[:,2]
        start_theta=theta_sample[0]
        sample_start_idx=-1
        sample_end_idx=-1
        for i in range(len(x_sample)):
            if abs(theta_sample[i]-start_theta)>0.0001 and sample_start_idx==-1:
                sample_start_idx=i
            if abs(theta_sample[i]-start_theta)<0.0001 and sample_start_idx!=-1 and sample_end_idx==-1:
                sample_end_idx=i
        print("sample_start_idx=",sample_start_idx)
        print("sample_end_idx=",sample_end_idx)

        lateral_change_lane_dis=[]
        longi_change_lane_dis=[]
        start_vec=[x_sample[1]-x_sample[0],y_sample[1]-y_sample[0]]
        start_unit_vec=start_vec/(pow((start_vec[0])**2+(start_vec[1])**2,0.5))
        start_to_startchange_vec=[x_sample[sample_start_idx]-x_sample[0],y_sample[sample_start_idx]-y_sample[0]]
        change_longi_base_val=abs(self.InnerProd(start_unit_vec,start_to_startchange_vec))
        for i in range(sample_start_idx,sample_end_idx+1):
            start_to_this_vec=[x_sample[i]-x_sample[0],y_sample[i]-y_sample[0]]
            longi=abs(self.InnerProd(start_unit_vec,start_to_this_vec))-change_longi_base_val
            lateral=abs(self.CrossProd(start_unit_vec,start_to_this_vec))
            lateral_change_lane_dis.append(lateral)
            longi_change_lane_dis.append(longi)

        self.lateral_dis_ratio=[]
        self.longi_dis_ratio=[]
        for i in range(len(lateral_change_lane_dis)):
            self.lateral_dis_ratio.append(lateral_change_lane_dis[i]/lateral_change_lane_dis[-1])
            self.longi_dis_ratio.append(longi_change_lane_dis[i]/longi_change_lane_dis[-1])


    def InnerProd(self,vec1,vec2):
        return vec1[0]*vec2[0]+vec1[1]*vec2[1]

    def CrossProd(self,vec1,vec2):
        return vec1[0]*vec2[1]-vec1[1]*vec2[0]

    def plan_curve(self,tar_yaw, tar_x, tar_y,change_lane_length,change_lane_lateral_x):

        start_ind, end_ind, tar_match_idx=self.cal_curve_start_tar_end_index(tar_yaw, tar_x, tar_y, change_lane_length)
        print("start idx=",start_ind)
        print("end ind=",end_ind)
        number_of_point=end_ind-start_ind+1
        number_of_variable = number_of_point*4
        l = cvxpy.Variable(number_of_variable)
        # import pdb;pdb.set_trace()
        cost = 0.0
        constrains = []

        #dl cost
        for i in range(0,number_of_point):
            cost += cvxpy.sum_squares(l[i*4+1])*10
        #ddl cost
        for i in range(0,number_of_point):
            cost += cvxpy.sum_squares(l[i*4+2])*10000
        #dddl cost
        for i in range(0,number_of_point):
            cost += cvxpy.sum_squares(l[i*4+3])*100000

        #积分链约束
        for i in range(0,number_of_point-1):
            ds=self.reference_line.ref_s[i+1]-self.reference_line.ref_s[i]
            constrains += [l[4*(i+1)+2]==l[4*i+2]+l[4*i+3]*ds]
            constrains += [l[4*(i+1)+1]==l[4*i+1]+l[4*i+2]*ds+0.5*l[4*i+3]*ds*ds]
            constrains += [l[4*(i+1)]==l[4*i]+l[4*i+1]*ds+0.5*l[4*i+2]*ds*ds+0.16667*l[4*i+3]*ds*ds*ds]

        # for i in range(0, number_of_variable):
        #     constrains += [
        #         self.reference_line.ref_k[start_ind + i] <= self.k_max - self.k_max * self.reference_line.renumf_k[
        #             start_ind + i] * l[i]]

        constrains += [l[(number_of_point-1)*4] == change_lane_lateral_x]
        # constrains += [l[number_of_variable - 1] == l[number_of_variable - 2]]
        constrains += [l[0] == 0]
        # constrains += [l[0] == l[1]]
        constrains += [l[1] == 0]
        constrains += [l[2] == 0]
        constrains += [l[3] == 0]
        constrains += [l[-1] == 0]
        constrains += [l[-2] == 0]
        constrains += [l[-3] == 0]

                # 途径点约束
        # constrains += [
        #     tar_x == self.reference_line.ref_x[tar_match_idx] - l[(tar_match_idx - start_ind)*4] * math.sin(
        #         self.reference_line.ref_yaw[tar_match_idx])]
        # constrains += [
        #     tar_y == self.reference_line.ref_y[tar_match_idx] + l[(tar_match_idx - start_ind)*4] * math.cos(
        #         self.reference_line.ref_yaw[tar_match_idx])]
        constrains += [tar_yaw
                       - tar_yaw * self.reference_line.ref_k[tar_match_idx] * l[(tar_match_idx - start_ind)*4]
                       == l[(tar_match_idx - start_ind)*4+1]
                       + self.reference_line.ref_yaw[tar_match_idx]
                       - self.reference_line.ref_yaw[tar_match_idx] * self.reference_line.ref_k[
                           tar_match_idx] * l[(tar_match_idx - start_ind)*4]]
        
        #途径点软约束
        cost += cvxpy.sum_squares((tar_x)-(self.reference_line.ref_x[tar_match_idx] - l[(tar_match_idx - start_ind)*4] * math.sin(
                self.reference_line.ref_yaw[tar_match_idx])))*1000
        cost += cvxpy.sum_squares((tar_y)-(self.reference_line.ref_y[tar_match_idx] + l[(tar_match_idx - start_ind)*4] * math.cos(
                self.reference_line.ref_yaw[tar_match_idx])))*1000
        cost += cvxpy.sum_squares((tar_yaw - tar_yaw * self.reference_line.ref_k[tar_match_idx] * l[(tar_match_idx - start_ind)*4])
                                  -(l[(tar_match_idx - start_ind)*4+1]
                                    + self.reference_line.ref_yaw[tar_match_idx]
                                    - self.reference_line.ref_yaw[tar_match_idx] * self.reference_line.ref_k[
                                    tar_match_idx] * l[(tar_match_idx - start_ind)*4]))*1000


        import time
        opt_time1=time.time()
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)
        opt_time2=time.time()
        opt_time=opt_time2-opt_time1
        print("opt_time=",opt_time)
        opt_result = l.value
        print('opt_result',opt_result)
        opt_l=[]
        for i in range(number_of_point):
            opt_l.append(opt_result[i*4])
        opt_dl=[]
        for i in range(number_of_point):
            opt_dl.append(opt_result[i*4+1])
        opt_ddl=[]
        for i in range(number_of_point):
            opt_ddl.append(opt_result[i*4+2])

        # 计算规划路段的笛卡尔坐标系轨迹
        x, y, yaw ,kappa= self.s_l_to_x_y(start_ind, end_ind, opt_l,opt_dl,opt_ddl)
        print("opt tar point : x=",x[tar_match_idx - start_ind]," y=",y[tar_match_idx - start_ind]," theta=",yaw[tar_match_idx - start_ind])

        reference_line_for_speed_planner_change_lane = Reference_line.Reference_line_for_speed_planner(x, y, yaw, kappa)

        # 对参考线切片得到变道之前的轨迹
        x_temp = self.reference_line.ref_x[0:start_ind - 1]
        y_temp = self.reference_line.ref_y[0:start_ind - 1]
        yaw_temp = self.reference_line.ref_yaw[0:start_ind - 1]
        kappa_temp = self.reference_line.ref_k[0:start_ind - 1]

        reference_line_for_speed_planner_pre_change_lane = Reference_line.Reference_line_for_speed_planner(x_temp,
                                                                                                           y_temp,
                                                                                                           yaw_temp,kappa_temp)
        
        # 对参考线切片得到变道之后的轨迹
        # 先要拿到变道后的点对应于目标参考线的idx
        change_lane_terminal_vec=[x[-2]-x[-1],y[-2]-y[-1]]
        tarrefline_start_idx=-1
        for i in range(len(self.tar_reference_line.ref_x)):
            opt_last_point_to_tarrefline_vec=[self.tar_reference_line.ref_x[i]-x[-1],self.tar_reference_line.ref_y[i]-y[-1]]
            inner_prod=self.InnerProd(change_lane_terminal_vec,opt_last_point_to_tarrefline_vec)
            if inner_prod<0:
                tarrefline_start_idx=i
                break
        x_temp = self.tar_reference_line.ref_x[tarrefline_start_idx:-1]
        y_temp = self.tar_reference_line.ref_y[tarrefline_start_idx:-1]
        yaw_temp = self.tar_reference_line.ref_yaw[tarrefline_start_idx:-1]
        kappa_temp = self.tar_reference_line.ref_k[tarrefline_start_idx:-1]

        reference_line_for_speed_planner_after_change_lane = Reference_line.Reference_line_for_speed_planner(x_temp,
                                                                                                           y_temp,
                                                                                                           yaw_temp,kappa_temp)

        # 拼接三段路径
        reference_line_for_speed_planner = self.joint_two_speed_reference_line(
            reference_line_for_speed_planner_pre_change_lane, reference_line_for_speed_planner_change_lane)
        reference_line_for_speed_planner=self.joint_two_speed_reference_line(reference_line_for_speed_planner,reference_line_for_speed_planner_after_change_lane)

        # 拼接后的轨迹就是最终规划要返回的轨迹
        x = reference_line_for_speed_planner.ref_x
        y = reference_line_for_speed_planner.ref_y
        yaw = reference_line_for_speed_planner.ref_yaw
        kappa = reference_line_for_speed_planner.ref_k
        # print(v)

        return x, y, yaw, kappa, reference_line_for_speed_planner, tar_match_idx
    


    # def plan_doublecurve(self, start_change_lane_length,change_lane_length,change_back_straight_length,change_lane_back_length):

    #     start_idx1, end_idx1, start_idx2, end_idx2=self.cal_double_curve_start_tar_end_index(start_change_lane_length
    #                                                                             ,change_lane_length
    #                                                                             ,change_back_straight_length
    #                                                                             ,change_lane_back_length)

    #     number_of_point=end_idx2-start_idx1+1
    #     number_of_variable = number_of_point*4
    #     l = cvxpy.Variable(number_of_variable)

    #     cost = 0.0
    #     constrains = []

    #     #dl cost
    #     for i in range(0,number_of_point):
    #         cost += cvxpy.sum_squares(l[i*4+1])*1
    #     #ddl cost
    #     for i in range(0,number_of_point):
    #         cost += cvxpy.sum_squares(l[i*4+2])*1
    #     #dddl cost
    #     for i in range(0,number_of_point):
    #         cost += cvxpy.sum_squares(l[i*4+3])*1000

    #     #积分链约束
    #     for i in range(0,number_of_point-1):
    #         ds=self.reference_line.ref_s[i+1]-self.reference_line.ref_s[i]
    #         constrains += [l[4*(i+1)+2]==l[4*i+2]+l[4*i+3]*ds]
    #         constrains += [l[4*(i+1)+1]==l[4*i+1]+l[4*i+2]*ds+0.5*l[4*i+3]*ds*ds]
    #         constrains += [l[4*(i+1)]==l[4*i]+l[4*i+1]*ds+0.5*l[4*i+2]*ds*ds+0.16667*l[4*i+3]*ds*ds*ds]

    #     # for i in range(0, number_of_variable):
    #     #     constrains += [
    #     #         self.reference_line.ref_k[start_ind + i] <= self.k_max - self.k_max * self.reference_line.renumf_k[
    #     #             start_ind + i] * l[i]]

    #     constrains += [l[(end_idx1-start_idx1)*4] == -self.lane_width]
    #     constrains += [l[(start_idx2-start_idx1)*4] == -self.lane_width]
    #     constrains += [l[(end_idx2-start_idx1)*4] == 0]
    #     # constrains += [l[number_of_variable - 1] == l[number_of_variable - 2]]
    #     constrains += [l[0] == 0]
    #     # constrains += [l[0] == l[1]]
    #     constrains += [l[1] == 0]
    #     constrains += [l[2] == 0]
    #     constrains += [l[3] == 0]

    #     constrains += [l[(end_idx1-start_idx1)*4+1] == 0]
    #     constrains += [l[(end_idx1-start_idx1)*4+2] == 0]
    #     constrains += [l[(end_idx1-start_idx1)*4+3] == 0]

    #     constrains += [l[(start_idx2-start_idx1)*4+1] == 0]
    #     constrains += [l[(start_idx2-start_idx1)*4+2] == 0]
    #     constrains += [l[(start_idx2-start_idx1)*4+3] == 0]


    #     constrains += [l[-1] == 0]
    #     constrains += [l[-2] == 0]
    #     constrains += [l[-3] == 0]

    #     import time
    #     opt_time1=time.time()
    #     prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    #     prob.solve(solver=cvxpy.OSQP)
    #     opt_time2=time.time()
    #     opt_time=opt_time2-opt_time1
    #     print("opt_time=",opt_time)
    #     opt_result = l.value
    #     print('opt_result',opt_result)
    #     opt_l=[]
    #     for i in range(number_of_point):
    #         opt_l.append(opt_result[i*4])

    #     # 计算规划路段的笛卡尔坐标系轨迹
    #     x, y, yaw = self.s_l_to_x_y(start_idx1, end_idx2, opt_l,opt_result)


    #     reference_line_for_speed_planner_change_lane = Reference_line.Reference_line_for_speed_planner(x, y, yaw)

    #     # 对参考线切片得到变道之前的轨迹
    #     x_temp = self.reference_line.ref_x[0:start_idx1 - 1]
    #     y_temp = self.reference_line.ref_y[0:start_idx1 - 1]
    #     yaw_temp = self.reference_line.ref_yaw[0:start_idx1 - 1]

    #     reference_line_for_speed_planner_pre_change_lane = Reference_line.Reference_line_for_speed_planner(x_temp,
    #                                                                                                        y_temp,
    #                                                                                                        yaw_temp)
        
    #     # 对参考线切片得到变道之后的轨迹
    #     # 先要拿到变道后的点对应于目标参考线的idx
    #     change_lane_terminal_vec=[x[-2]-x[-1],y[-2]-y[-1]]
    #     tarrefline_start_idx=-1
    #     for i in range(len(self.reference_line.ref_x)):
    #         opt_last_point_to_tarrefline_vec=[self.reference_line.ref_x[i]-x[-1],self.reference_line.ref_y[i]-y[-1]]
    #         inner_prod=self.InnerProd(change_lane_terminal_vec,opt_last_point_to_tarrefline_vec)
    #         if inner_prod<0:
    #             tarrefline_start_idx=i
    #             break
    #     x_temp = self.reference_line.ref_x[tarrefline_start_idx:-1]
    #     y_temp = self.reference_line.ref_y[tarrefline_start_idx:-1]
    #     yaw_temp = self.reference_line.ref_yaw[tarrefline_start_idx:-1]

    #     reference_line_for_speed_planner_after_change_lane = Reference_line.Reference_line_for_speed_planner(x_temp,
    #                                                                                                        y_temp,
    #                                                                                                        yaw_temp)

    #     # 拼接三段路径
    #     reference_line_for_speed_planner = self.joint_two_speed_reference_line(
    #         reference_line_for_speed_planner_pre_change_lane, reference_line_for_speed_planner_change_lane)
    #     reference_line_for_speed_planner=self.joint_two_speed_reference_line(reference_line_for_speed_planner,reference_line_for_speed_planner_after_change_lane)

    #     # 拼接后的轨迹就是最终规划要返回的轨迹
    #     x = reference_line_for_speed_planner.ref_x
    #     y = reference_line_for_speed_planner.ref_y
    #     yaw = reference_line_for_speed_planner.ref_yaw

    #     # print(v)

    #     return x, y, yaw, reference_line_for_speed_planner
    
    def NormorlizeAngle(self,angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def s_l_to_x_y(self, start_ind, end_ind, opt_l,opt_dl,opt_ddl):
        x = []
        y = []
        yaw = []
        kappa=[]
        for i in range(start_ind, end_ind + 1):
            x_temp = self.reference_line.ref_x[i] - opt_l[i - start_ind] * math.sin(self.reference_line.ref_yaw[i])
            y_temp = self.reference_line.ref_y[i] + opt_l[i - start_ind] * math.cos(self.reference_line.ref_yaw[i])
            yaw_temp=math.atan(opt_dl[i - start_ind]/(1-self.reference_line.ref_k[i]*opt_l[i - start_ind]))+self.reference_line.ref_yaw[i]
            yaw_temp=self.NormorlizeAngle(yaw_temp)
            #这里设计的参考线曲率不变，d_kr=0
            AA=opt_ddl[i - start_ind]+self.reference_line.ref_k[i]*opt_dl[i - start_ind]*math.tan(yaw_temp-self.reference_line.ref_yaw[i])
            BB=pow(math.cos(yaw_temp-self.reference_line.ref_yaw[i]),2)/(1-self.reference_line.ref_k[i]*opt_l[i - start_ind])
            CC=math.cos(yaw_temp-self.reference_line.ref_yaw[i])/(1-self.reference_line.ref_k[i]*opt_l[i - start_ind])
            k_temp=(AA*BB+self.reference_line.ref_k[i])*CC
            x.append(x_temp)
            y.append(y_temp)
            yaw.append(yaw_temp)
            kappa.append(k_temp)

        return x, y, yaw, kappa
    
    def InnerProd(self,x1,x2):
        return x1[0]*x2[0]+x1[1]*x2[1]

    def joint_two_speed_reference_line(self, line1, line2):
        x = line1.ref_x + line2.ref_x
        y = line1.ref_y + line2.ref_y
        yaw = line1.ref_yaw + line2.ref_yaw
        kappa = line1.ref_k+line2.ref_k
        speed_ref_line = Reference_line.Reference_line_for_speed_planner(x, y, yaw, kappa)
        return speed_ref_line

    def cal_curve_start_tar_end_index(self,tar_yaw, tar_x, tar_y,change_lane_length):
        tar_match_idx=-1
        min_dis2=999999
        for i in range(len(self.reference_line.ref_x)):
            dis2=(self.reference_line.ref_x[i]-tar_x)**2+(self.reference_line.ref_y[i]-tar_y)**2
            if dis2<min_dis2:
                min_dis2=dis2
                tar_match_idx=i

        match_refline_vec=[self.reference_line.ref_x[tar_match_idx+1]-self.reference_line.ref_x[tar_match_idx]
                           ,self.reference_line.ref_y[tar_match_idx+1]-self.reference_line.ref_y[tar_match_idx]]
        match_refline_unit_vec=match_refline_vec/(pow((match_refline_vec[0])**2+(match_refline_vec[1])**2,0.5))
        match_to_targetpoint=[tar_x-self.reference_line.ref_x[tar_match_idx],tar_y-self.reference_line.ref_y[tar_match_idx]]
        lateral_dis=abs(self.CrossProd(match_refline_unit_vec,match_to_targetpoint))
        lateral_ratio=np.clip(lateral_dis/self.lane_width,0,1)
        sample_table_match_idx=-1
        for i in range(len(self.lateral_dis_ratio)):
            if lateral_ratio < self.lateral_dis_ratio[i]:
                sample_table_match_idx=i
                break
        longi_ratio=self.longi_dis_ratio[sample_table_match_idx]
        longi_dis=longi_ratio*change_lane_length
        rest_longi_dis=change_lane_length-longi_dis

        start_change_lane_idx=-1
        end_change_lane_idx=-1
        # import pdb;pdb.set_trace()
        #找开始变道的索引
        ii=tar_match_idx
        tt_dis=0.0
        while ii>=1:
            tt_dis+=pow((self.reference_line.ref_x[ii]-self.reference_line.ref_x[ii-1])**2
                        +(self.reference_line.ref_y[ii]-self.reference_line.ref_y[ii-1])**2,0.5)
            if tt_dis>longi_dis:
                break
            ii-=1
        start_change_lane_idx=ii

        #找结束变道的索引
        ii=tar_match_idx
        tt_dis=0.0
        while ii<len(self.reference_line.ref_x)-1:
            tt_dis+=pow((self.reference_line.ref_x[ii]-self.reference_line.ref_x[ii+1])**2
                        +(self.reference_line.ref_y[ii]-self.reference_line.ref_y[ii+1])**2,0.5)
            if tt_dis>rest_longi_dis:
                break
            ii+=1
        end_change_lane_idx=ii



        return start_change_lane_idx,end_change_lane_idx, tar_match_idx
    


    def cal_double_curve_start_tar_end_index(self,start_change_lane_length
                                    ,change_lane_length
                                    ,change_back_straight_length
                                    ,change_lane_back_length):
        start_idx1=-1
        end_idx1=-1
        start_idx2=-1
        end_idx2=-1
        for i,s in enumerate(self.reference_line.ref_s):
            if start_idx1==-1 and  s>start_change_lane_length:
                start_idx1=i
            if end_idx1==-1 and  s>start_change_lane_length+change_lane_length:
                end_idx1=i
            if start_idx2==-1 and  s>start_change_lane_length+change_lane_length+change_back_straight_length:
                start_idx2=i
            if end_idx2==-1 and  s>start_change_lane_length+change_lane_length+change_back_straight_length+change_lane_back_length:
                end_idx2=i
        return start_idx1,end_idx1,start_idx2,end_idx2

    def cal_length_of_change_lane(self,obs_v):
        length_of_change_lane=30+obs_v*0.1
        return length_of_change_lane

    def cal_start_to_tar_to_end_s(self,d_to_main_lane,d_to_tar_lane,length_of_change_lane):
        s_start_to_tar=length_of_change_lane*(d_to_main_lane/(d_to_main_lane+d_to_tar_lane))
        s_tar_to_end=length_of_change_lane*(d_to_tar_lane/(d_to_main_lane+d_to_tar_lane))
        return s_start_to_tar,s_tar_to_end


class Speed_Planner:
    def __init__(self, reference_line):
        self.reference_line = reference_line
        self.d_t = P.P.speed_planner_d_t

    def plan(self, tar_s, tar_time, default_v, now_v, now_ind):

        #加这段是因为过了目标点后，main函数中调用speed plan时的参数tar_s和tar_time都会是负数，这个时候number of variable就没有了，所以就规划不出来速度
        #其实这里写的number_of_variable这个变量挺随便的，直接按照目标点的时间算了，其实很离谱
        if self.reference_line.ref_s[now_ind] >= tar_s:
            v=[]
            for i in range(now_ind,len(self.reference_line.ref_s)):
                v.append(default_v)
            return v


        number_of_variable = int(tar_time * 1.5 / self.d_t)

        tar_ind = int(tar_time / self.d_t)

        s = cvxpy.Variable(number_of_variable)

        cost = 0.0
        constrains = []

        for i in range(1, number_of_variable - 1):
            cost += cvxpy.sum_squares((s[i] - s[i - 1]) / self.d_t - default_v)  # 偏离预期速度代价
            cost += cvxpy.sum_squares((s[i + 1] + s[i - 1] - 2 * s[i]) / (self.d_t ** 2))  # 加速度代价
            constrains += [s[i] >= s[i - 1]]  # 往前走约束
        constrains += [s[number_of_variable - 1] >= (self.reference_line.ref_s[-1]-self.reference_line.ref_s[now_ind])]  # 最后至少要到达终点约束
        print("tar_s=", tar_s)
        print("tar_ind=", tar_ind)
        constrains += [s[tar_ind] == tar_s]  # 经过点约束
        constrains += [s[0] == 0]  # 起点约束
        constrains += [now_v == (s[1] - s[0]) / self.d_t]  # 起点速度约束

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)
        s_plan = s.value

        # 画出s_t图
        t = []
        time = 0.0
        for i in range(0, len(s_plan)):
            t.append(time)
            time = time + 0.1

        for i in range(0,len(s_plan)):
            if s_plan[i]>=tar_s:
                print("这个周期到达s的时间是:",i*0.1,",理想时间是:",tar_time,"实际s是",s_plan[i],"预瞄的s是",tar_s)
                break


        v = []
        matching_s_ind = now_ind

        for i in range(0, len(s_plan)):
            if (self.reference_line.ref_s[matching_s_ind] - self.reference_line.ref_s[now_ind]) <= s_plan[i]:
                if i == 0:
                    v.append((s_plan[i + 1] - s_plan[i]) / self.d_t)
                else:
                    v.append((s_plan[i] - s_plan[i - 1]) / self.d_t)
                matching_s_ind = matching_s_ind + 1
                # 避免起速太低启动不了的问题，可是这样的话起步的那次速度规划局就不准，但是也无所谓，因为到最后的版本速度规划会迭代进行
                if v[-1] < 1 and now_ind == 0:
                    v[-1] = 1
            if matching_s_ind >= len(self.reference_line.ref_x) :
                break
        print("v=====", v)
        return v
