import Car
import MPC
import P
import Planner
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Polygon
import Reference_line as ref
import math
import draw
import os
import sys
sys.path.append('/home/fengmq/code/test_vehicle_planner_kappa_v_limit/build')
sys.path.append('/home/fengmq/code/test_vehicle_planner_kappa_v_limit/build/proto')
import speed_planner_pybind
speed_planner_pybind.TestPybind(3.14)
print("success import pybind")
import speed_planner_msg_pb2
import speed_planner_config_pb2
from google.protobuf import text_format

def CrossProd(x1,x2):
    return x1[0]*x2[1]-x1[1]*x2[0]


def cal_k(x, y):
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
        a2b=[x[b_index]-x[a_index],y[b_index]-y[a_index]]
        a2c=[x[c_index]-x[a_index],y[c_index]-y[a_index]]
        crossprod=CrossProd(a2b,a2c)
        if crossprod>0:
            k_temp=abs(k_temp)
        else:
            k_temp=-abs(k_temp) 
        k.append(k_temp)
        # print("第",i+1,"个k=",k_temp)
    k.append(0)
    return k


def pi_2_pi(theta):
    while theta > 3.1415:
        theta -= 2.0 * 3.1415

    while theta < -3.1415:
        theta += 2.0 * 3.1415

    return theta


def cal_tar_time(main_car, R_tar, tar_x, tar_y):
    longi_dis=50
    min_dis2=9999999
    match_idx=-1
    for i in range(len(R_tar.ref_x)):
        dis2=(main_car.x-R_tar.ref_x[i])**2+(main_car.y-R_tar.ref_y[i])**2
        if dis2<min_dis2:
            min_dis2=dis2
            match_idx=i
    tar_point_min_dis2=9999999
    tar_point_match_idx=-1
    for i in range(len(R_tar.ref_x)):
        dis2=(tar_x-R_tar.ref_x[i])**2+(tar_y-R_tar.ref_y[i])**2
        if dis2<tar_point_min_dis2:
            tar_point_min_dis2=dis2
            tar_point_match_idx=i

    main_car_s=R_tar.ref_s[match_idx]
    tar_point_s=R_tar.ref_s[tar_point_match_idx]

    # import pdb;pdb.set_trace()
    return (tar_point_s - main_car_s-longi_dis) / main_car.v

def trajectory_plan_determine(x,y,yaw,tar_x,tar_y,tar_yaw):
    min_d=100000
    match_index=0
    for i in range(0,len(x)):
        if((x[i]-tar_x)**2+(y[i]-tar_y)**2)<min_d:
            min_d=((x[i]-tar_x)**2+(y[i]-tar_y)**2)
            match_index=i
    e_x=x[match_index]-tar_x
    e_y=y[match_index]-tar_y
    e_yaw=yaw[match_index]-tar_yaw
    print("目标x的误差是",e_x,"目标y的误差是", e_y,"目标yaw的误差是" ,e_yaw)

def cal_vehicle_pts(x,y,theta,width,long):
    pts=[[-long/2,width/2],[long/2,width/2],[long/2,-width/2],[-long/2,-width/2]]
    pt_list_res=[]
    rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]])
    for pt in pts:
        local_points = np.array([pt[0], pt[1]])
        rotated_point = np.dot(rotation_matrix, local_points)
        world_point = rotated_point + np.array([x, y])
        pt_list_res.append((world_point[0],world_point[1]))
    return pt_list_res

def GetSpeedPlannerConfig():
    config_path = '/home/fengmq/code/test_vehicle_planner_kappa_v_limit/proto_data/speed_planner_config.pb.txt'
    config = speed_planner_config_pb2.SpeedPlannerConfig()
    with open(config_path, 'r') as file:
        file_content = file.read()
        res = text_format.Parse(file_content, config)
    return config

def NormalizeAnlge(angle):
    while angle>math.pi:
        angle-=2*math.pi
    while angle<-math.pi:
        angle+=2*math.pi
    return angle

def GenerateMainCarTraj(R_tar,main_car_v):
    test_car_speed_planner_delta_t=GetSpeedPlannerConfig().delta_time
    ds=test_car_speed_planner_delta_t*main_car_v
    path_idx=1
    total_s=0
    traj_x=[]
    traj_y=[]
    traj_theta=[]
    while path_idx<len(R_tar.ref_x)-1:
        total_s+=ds
        while path_idx<len(R_tar.ref_s)-1 and total_s>R_tar.ref_s[path_idx]:
            path_idx+=1
        # if path_idx==len(R_tar.ref_s):
        #     break
        x_pre=R_tar.ref_x[path_idx-1]
        x_after=R_tar.ref_x[path_idx]
        y_pre=R_tar.ref_y[path_idx-1]
        y_after=R_tar.ref_y[path_idx]
        theta_pre=R_tar.ref_yaw[path_idx-1]
        theta_after=R_tar.ref_yaw[path_idx]
        ratio=(total_s-R_tar.ref_s[path_idx-1])/(R_tar.ref_s[path_idx]-R_tar.ref_s[path_idx-1])
        x_temp=x_pre+ratio*(x_after-x_pre)
        y_temp=y_pre+ratio*(y_after-y_pre)
        theta_temp=NormalizeAnlge(theta_pre+ratio*NormalizeAnlge(theta_after-theta_pre))
        traj_x.append(x_temp)
        traj_y.append(y_temp)
        traj_theta.append(theta_temp)
    return traj_x,traj_y,traj_theta


def RunSpeedPlanner(x,y,yaw,kappa,test_car,tar_time,tar_match_point_index,test_car_tar_v,test_car_v):
    match_idx=-1
    min_dis2=float('inf')
    for i in range(len(x)-1):
        temp_dis2=(x[i]-test_car.x)**2+(y[i]-test_car.y)**2
        if temp_dis2<min_dis2:
            min_dis2=temp_dis2
            match_idx=i
    x_forward=[]
    y_forward=[]
    theta_forward=[]
    s_forward=[]
    kappa_forward=[]
    s_now=0.0
    match_pt2_next_vec=[x[match_idx+1]-x[match_idx],y[match_idx+1]-y[match_idx]]
    match_pt2_ego_vec=[test_car.x-x[match_idx],test_car.y-y[match_idx]]
    inner_prod=match_pt2_next_vec[0]*match_pt2_ego_vec[0]+match_pt2_next_vec[1]*match_pt2_ego_vec[1]

    start_idx=match_idx+1 if inner_prod>0 else match_idx
    x_forward.append(test_car.x)
    y_forward.append(test_car.y)
    theta_forward.append(test_car.yaw)
    s_forward.append(s_now)
    kappa_forward.append(0.0)
    for i in range(start_idx,len(x)):
        s_now+=math.sqrt((x[i]-x_forward[-1])**2+(y[i]-y_forward[-1])**2)
        x_forward.append(x[i])
        y_forward.append(y[i])
        theta_forward.append(yaw[i])
        s_forward.append(s_now)
        kappa_forward.append(kappa[i])
    kappa_forward[0]=kappa_forward[1]

    tar_s=0.0
    # import pdb;pdb.set_trace()
    # if (tar_match_point_index>match_idx) or ((tar_match_point_index==match_idx) and (inner_prod<0)):
    #     tar_s=s_forward[int(tar_match_point_index-start_idx+1)]
    if tar_time!=0.0:
        tar_s=s_forward[int(tar_match_point_index-start_idx+1)]
    print("tar_s=",tar_s)
    proto_msg=speed_planner_msg_pb2.SpeedPlannerProtoMsg()
    proto_msg.init_v=test_car_v
    proto_msg.init_a=0.0
    proto_msg.init_jerk=0.0
    for i in range(len(x_forward)):
        temp_pt=speed_planner_msg_pb2.OptPathPoint()
        temp_pt.x=x_forward[i]
        temp_pt.y=y_forward[i]
        temp_pt.theta=theta_forward[i]
        temp_pt.s=s_forward[i]
        temp_pt.kappa=kappa_forward[i]
        proto_msg.opt_path_points.add().CopyFrom(temp_pt)
    # import pdb;pdb.set_trace()
    config=GetSpeedPlannerConfig()
    config.target_time=tar_time
    config.target_s=tar_s
    config.target_v=test_car_tar_v
    print("success load parmas") 
    proto_msg_res_str=speed_planner_pybind.RunSpeedPlanner(proto_msg.SerializeToString(),config.SerializeToString())
    proto_msg_res=speed_planner_msg_pb2.SpeedPlannerProtoMsg()
    proto_msg_res.ParseFromString(proto_msg_res_str)

    return proto_msg_res

#move_dis为正时向外扩
def MoveCurveRefline(refline,circle_center,move_dis):
    x=[]
    y=[]
    for i in range(len(refline.ref_x)):
        center2point=[refline.ref_x[i]-circle_center[0],refline.ref_y[i]-circle_center[1]]
        center2point_unit_vec=center2point/pow((center2point[0])**2+(center2point[1])**2,0.5)
        center2newpoint=center2point+move_dis*center2point_unit_vec
        new_point=center2newpoint+circle_center
        x.append(new_point[0])
        y.append(new_point[1])
    new_r=ref.Reference_line(x, y, 1)
    return new_r

def GenerateReflineAndRb(lane_width, scenario, curve_r,to_left):
    tar_x=0
    tar_y=0
    tar_yaw=0
    if scenario==1:
        x = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110,400]
        y = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,5]
        R = ref.Reference_line(x, y, 1)
        y_tar = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,1.5]
        R_tar = ref.Reference_line(x, y_tar, 1)
        y_lb1=[6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75,6.75]
        lb1=ref.Reference_line(x, y_lb1, 1)
        y_lb2=[3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25,3.25]
        lb2=ref.Reference_line(x, y_lb2, 1)
        y_lb3=[-0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25,-0.25]
        lb3=ref.Reference_line(x, y_lb3, 1)

        test_car_tar_v=15
        main_car_v=12

        if to_left:
            temp_refline=R
            R=R_tar
            R_tar=temp_refline
            change_lane_lateral_x=lane_width
            tar_x=160
            tar_y=4.9
            tar_yaw=0.05
        else:
            change_lane_lateral_x=-lane_width
            tar_x=160
            tar_y=1.6
            tar_yaw=-0.02

        return R,R_tar,lb1,lb2,lb3,change_lane_lateral_x,tar_x,tar_y,tar_yaw,test_car_tar_v,main_car_v
    
    elif scenario==2:
        num_of_ori_pt=30
        d_angle=0.0523
        circle_center=[0,curve_r]
        x=[]
        y=[]
        angle=-math.pi/2
        for i in range(num_of_ori_pt):
            x.append(circle_center[0]+curve_r*math.cos(angle))
            y.append(circle_center[1]+curve_r*math.sin(angle))
            angle+=d_angle
        R = ref.Reference_line(x, y, 1)
        R_tar = MoveCurveRefline(R,circle_center,3.5)
        lb1=MoveCurveRefline(R,circle_center,-1.75)
        lb2=MoveCurveRefline(R,circle_center,1.75)
        lb3=MoveCurveRefline(R_tar,circle_center,1.75)

        test_car_tar_v=17
        main_car_v=13
        if to_left:
            temp_refline=R
            R=R_tar
            R_tar=temp_refline
            change_lane_lateral_x=lane_width
            tar_x=100
            tar_y=20.3
            tar_yaw=0.4
        else:
            change_lane_lateral_x=-lane_width
            tar_x=100
            tar_y=18
            tar_yaw=0.38


        return R,R_tar,lb1,lb2,lb3,change_lane_lateral_x,tar_x,tar_y,tar_yaw,test_car_tar_v,main_car_v
    
    elif scenario==3:
        x = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110,400]
        y = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,5]
        R = ref.Reference_line(x, y, 1)
        R_tar = R
        y_lb1=[6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75, 6.75,6.75]
        lb1=ref.Reference_line(x, y_lb1, 1)
        y_lb2=[3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25, 3.25,3.25]
        lb2=ref.Reference_line(x, y_lb2, 1)
        change_lane_lateral_x=0.0
        y_lb3=[-0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25,-0.25]
        lb3=ref.Reference_line(x, y_lb3, 1)

        test_car_tar_v=20
        main_car_v=18

        tar_x=120
        tar_y=5
        tar_yaw=0


        return R,R_tar,lb1,lb2,lb3,change_lane_lateral_x,tar_x,tar_y,tar_yaw,test_car_tar_v,main_car_v
    
    elif scenario==4:
        num_of_ori_pt=20
        d_angle=0.0523
        circle_center=[0,curve_r]
        x=[]
        y=[]
        angle=-math.pi/2
        for i in range(num_of_ori_pt):
            x.append(circle_center[0]+curve_r*math.cos(angle))
            y.append(circle_center[1]+curve_r*math.sin(angle))
            angle+=d_angle
        R = ref.Reference_line(x, y, 1)
        R_tar = R
        lb1=MoveCurveRefline(R,circle_center,-1.75)
        lb2=MoveCurveRefline(R,circle_center,1.75)
        lb3=MoveCurveRefline(R_tar,circle_center,1.75)

        change_lane_lateral_x=0.0
        test_car_tar_v=20
        main_car_v=18

        tar_x=100
        tar_y=26.3
        tar_yaw=0.5


        return R,R_tar,lb1,lb2,lb3,change_lane_lateral_x,tar_x,tar_y,tar_yaw,test_car_tar_v,main_car_v


# 初始化两辆车
# test_car = Car.Car(0, 5, 0, P.P.test_car_start_v)                   #直道
# main_car = Car.Car(0, 1.5, 0, P.P.main_car_start_v)



# 初始化道路参考线
lane_width=3.5

R,R_tar,lb1,lb2,lb3,change_lane_lateral_x,tar_x,tar_y,tar_yaw,test_car_tar_v,main_car_v=GenerateReflineAndRb(lane_width,1,250,True)

ax_x_low=np.min(R.ref_x)+50
# ax_x_high=np.max(R.ref_x)/1.5+5
ax_x_high=175
ax_y_low=np.min(R.ref_y)-5
# ax_y_high=np.max(R.ref_y)/1.5+5
ax_y_high=10

test_car = Car.Car(R.ref_x[0], R.ref_y[0], R.ref_yaw[0], test_car_tar_v)                   
main_car = Car.Car(R_tar.ref_x[0], R_tar.ref_y[0], R_tar.ref_yaw[0], main_car_v)

change_lane_length=test_car_tar_v*8

# 初始化目标
# tar_x = 65
# tar_y =3.50
# tar_yaw=-0.15
# plt.plot(R.ref_x,R.ref_y)
# plt.pause(100)
# tar_x = 51
# tar_y =5
# tar_yaw=-0

1
# 规划test车路径和速度
planner = Planner.Planner(R, R_tar, lane_width,test_car)
x, y, yaw, kappa, reference_line_for_speed_planner, tar_match_point_index = planner.plan_curve(tar_yaw, tar_x, tar_y, change_lane_length,change_lane_lateral_x)
# k = cal_k(x, y)
trajectory_plan_determine(x,y,yaw,tar_x,tar_y,tar_yaw)

# 速度规划
speed_planner_config=GetSpeedPlannerConfig()
tar_time = cal_tar_time(main_car, R_tar, tar_x, tar_y)

if tar_time>=speed_planner_config.total_time or tar_time<0.0 :
    tar_time=0.0
proto_msg_res=RunSpeedPlanner(x,y,yaw,kappa,test_car,tar_time,tar_match_point_index,test_car_tar_v,test_car.v)
x_traj=[]
y_traj=[]
theta_traj=[]
v_traj=[]
k_traj=[]
for traj_pt in proto_msg_res.traj_points:
    x_traj.append(traj_pt.x)
    y_traj.append(traj_pt.y)
    theta_traj.append(traj_pt.theta)
    v_traj.append(traj_pt.v)
    k_traj.append(traj_pt.kappa)
# k_traj=cal_k(x_traj, y_traj)
# import pdb;pdb.set_trace()


# 获得主车的路径规划结果
# x_main_car = R_tar.ref_x
# y_main_car = R_tar.ref_y
# k_main_car = cal_k(x_main_car, y_main_car)
# yaw_main_car = R_tar.ref_yaw
# v_main_car = []
# for i in range(0, len(x_main_car)):
#     v_main_car.append(4)
x_main_car,y_main_car,yaw_main_car=GenerateMainCarTraj(R_tar,main_car_v)
k_main_car = cal_k(x_main_car, y_main_car)
v_main_car = []
for i in range(0, len(x_main_car)):
    v_main_car.append(main_car_v)

#这三种控制器写的逻辑不太一样。mpc控制器的参考路径是初始化的时候就给的。ilqr控制器是在实例化之后用update_trajectory函数更新路径。ilqr的参考路径是调用ilqr控制主函数时，通过参数target给的。

# 初始化mpc控制器
test_car_mpc = MPC.MPC(x_traj, y_traj, theta_traj, k_traj, v_traj)
main_car_mpc = MPC.MPC(x_main_car, y_main_car, yaw_main_car, k_main_car, v_main_car)



#选择控制器
controller="mpc"

#初始化控制器的参考轨迹
if controller == "mpc":
    test_car_mpc.update_trajectory(x_traj, y_traj, theta_traj, k_traj, v_traj)
    main_car_mpc.update_trajectory(x_main_car, y_main_car, yaw_main_car, k_main_car, v_main_car)


a_test_car = 0.0
a_main_car = 0.0
delta_test_car = 0.0
delta_main_car = 0.0

time = 0.0

rec_x_test = [test_car.x]
rec_y_test = [test_car.y]
rec_yaw_test = [test_car.yaw]
rec_v_test = [test_car.v]
rec_t_test = [0.0]
rec_x_main = [main_car.x]
rec_y_main = [main_car.y]
rec_yaw_main = [main_car.yaw]
rec_v_main = [main_car.v]
rec_t_main = [0.0]

test_car_traj_list=[]
test_car_pos_list=[]   #x,y,theta,steer
main_car_pos_list=[]


sim_step=3
rollout_sim_test_car_idx=0
rollout_sim_main_car_idx=0

cishu = 0
while True:
    cishu += 1
    match_idx_ego=-1
    min_dis2=float('inf')
    for i in range(len(x)):
        temp_dis2=(x[i]-test_car.x)**2+(y[i]-test_car.y)**2
        if temp_dis2<min_dis2:
            min_dis2=temp_dis2
            match_idx_ego=i
    if match_idx_ego==len(x)-10:
        break
    if cishu % 5 == 0:
        tar_time = cal_tar_time(main_car, R_tar, tar_x, tar_y)
        if tar_time>=speed_planner_config.total_time or tar_time<0.0 :
            tar_time=0.0

        proto_msg_res=RunSpeedPlanner(x,y,yaw,kappa,test_car,tar_time,tar_match_point_index,test_car_tar_v,test_car.v)
        rollout_sim_test_car_idx=0
        x_traj=[]
        y_traj=[]
        theta_traj=[]
        v_traj=[]
        k_traj=[]
        for traj_pt in proto_msg_res.traj_points:
            x_traj.append(traj_pt.x)
            y_traj.append(traj_pt.y)
            theta_traj.append(traj_pt.theta)
            v_traj.append(traj_pt.v)
            k_traj.append(traj_pt.kappa)
        if tar_time>0.1:
            x_tar_time=x_traj[int(tar_time/0.1)-1]
            y_tar_time=y_traj[int(tar_time/0.1)-1]
            longi_tar_dis=math.pow((x_tar_time-tar_x)**2+(y_tar_time-tar_y)**2,0.5)
            print("longi_tar_dis=",longi_tar_dis)
        # import pdb;pdb.set_trace()
        # k_traj=cal_k(x_traj, y_traj)

        if controller=="mpc":
            test_car_mpc.update_trajectory(x_traj,y_traj,theta_traj,k_traj,v_traj)

    # delta_test_car, a_test_car = test_car_mpc.cal_control(test_car)
    # delta_main_car, a_main_car = main_car_mpc.cal_control(main_car)
    # test_car.update(a_test_car, delta_test_car, 1)
    # main_car.update(a_main_car, delta_main_car, 1)
    # if controller == "mpc":
    #     delta_test_car, a_test_car , a_test_car_pid= test_car_mpc.cal_control(test_car)
    #     delta_main_car, a_main_car , a_main_car_pid = main_car_mpc.cal_control(main_car)

    # test_car.update(a_test_car_pid, delta_test_car, 1)
    # main_car.update(a_main_car_pid, delta_main_car, 1)

    rollout_sim_test_car_idx+=sim_step
    rollout_sim_main_car_idx+=sim_step
    if rollout_sim_main_car_idx>=len(main_car_mpc.path.cx):
        break
    if rollout_sim_test_car_idx>=len(test_car_mpc.path.cx):
        break
    test_car.UpdateWithoutControl(test_car_mpc.path.cx[rollout_sim_test_car_idx]
                                  , test_car_mpc.path.cy[rollout_sim_test_car_idx]
                                  , test_car_mpc.path.cyaw[rollout_sim_test_car_idx],
                                  test_car_mpc.v[rollout_sim_test_car_idx])
    main_car.UpdateWithoutControl(main_car_mpc.path.cx[rollout_sim_main_car_idx]
                                  , main_car_mpc.path.cy[rollout_sim_main_car_idx]
                                  , main_car_mpc.path.cyaw[rollout_sim_main_car_idx]
                                  ,main_car_mpc.v[rollout_sim_main_car_idx])

    print("==================================")
    print("ego x=",test_car.x," ,ego y=",test_car.y,",ego v=",test_car.v)
    print("test car acc=",a_test_car," , main car acc=",a_main_car)

    rec_x_test.append(test_car.x)
    rec_y_test.append(test_car.y)
    rec_yaw_test.append(test_car.yaw)
    rec_v_test.append(test_car.v)
    rec_t_test.append(time)
    rec_x_main.append(main_car.x)
    rec_y_main.append(main_car.y)
    rec_yaw_main.append(main_car.yaw)
    rec_v_main.append(main_car.v)
    rec_t_main.append(time)

    test_car_dy = (test_car.yaw - rec_yaw_test[-2]) / (test_car.v * P.P.dt+0.0001)
    test_car_steer = pi_2_pi(-math.atan(P.P.WB * test_car_dy))
    main_car_dy = (main_car.yaw - rec_yaw_main[-2]) / (main_car.v * P.P.dt+0.0001)
    main_car_steer = pi_2_pi(-math.atan(P.P.WB * main_car_dy))

    #生成画出目标点的线
    x_tar_point=[]
    y_tar_point=[]
    len_of_tar_point=3
    interval_of_draw=0.5
    len_of_draw=int(len_of_tar_point/interval_of_draw)
    for i in range(0,len_of_draw):
        x_tar_point.append(tar_x+math.cos(tar_yaw)*i*interval_of_draw)
        y_tar_point.append(tar_y+math.sin(tar_yaw)*i*interval_of_draw)

    test_car_traj=[]
    for i in range(len(x_traj)):
        test_car_traj.append([x_traj[i],y_traj[i],theta_traj[i],v_traj[i],k_traj[i]])
    test_car_traj_list.append(test_car_traj)
    print("len(test_car_traj_list)=",len(test_car_traj_list))

    test_car_pos_list.append([test_car.x,test_car.y,test_car.yaw,test_car_steer])
    main_car_pos_list.append([main_car.x,main_car.y,main_car.yaw,main_car_steer])


    time += P.P.dt






    


fig, ax = plt.subplots()
ax.set_aspect('equal')
# ax.set_xlim(np.min(R.ref_x)-5,np.max(R.ref_x)+5)
# ax.set_ylim(np.min(R.ref_y)-5,np.max(R.ref_y)+5)
ax.set_xlim(ax_x_low,ax_x_high)
ax.set_ylim(ax_y_low,ax_y_high)

test_car_traj_patch_list=[]
test_car_patch_list=[]
main_car_patch_list=[]
refline_patch_list=[]
lb_patch_list=[]

#generate traj patch
for test_car_traj in test_car_traj_list:
    polygen_vec=[]
    for i,traj_pt in enumerate(test_car_traj):
        opacity = (i + 1) / len(test_car_traj)  # 让透明度随着时间增加而增加
        width = 1
        long = 2
        vehicle_pts=cal_vehicle_pts(traj_pt[0] , traj_pt[1],traj_pt[2],width,long)
        rect = Polygon(vehicle_pts,
                        closed=True, color='blue', alpha=1-opacity)  
        polygen_vec.append(rect)
        ax.add_patch(rect)
        rect.set_visible(False)
    test_car_traj_patch_list.append(polygen_vec)


#generate vehicle pos patch
for test_car_pos in test_car_pos_list:
    test_car_patch=draw.CarPatch(test_car_pos[0],test_car_pos[1],test_car_pos[2],0,P.P,color='black')
    test_car_patch.add_to_axes(ax)
    test_car_patch.car_poly.set_visible(False)
    test_car_patch.fl_wheel_poly.set_visible(False)
    test_car_patch.fr_wheel_poly.set_visible(False)
    test_car_patch.rl_wheel_poly.set_visible(False)
    test_car_patch.rr_wheel_poly.set_visible(False)
    test_car_patch_list.append(test_car_patch)

# for main_car_pos in main_car_pos_list:
#     main_car_patch=draw.CarPatch(main_car_pos[0],main_car_pos[1],main_car_pos[2],0,P.P,color='black')
#     main_car_patch.add_to_axes(ax)
#     main_car_patch.car_poly.set_visible(False)
#     main_car_patch.fl_wheel_poly.set_visible(False)
#     main_car_patch.fr_wheel_poly.set_visible(False)
#     main_car_patch.rl_wheel_poly.set_visible(False)
#     main_car_patch.rr_wheel_poly.set_visible(False)
#     main_car_patch_list.append(main_car_patch)

#generate target pos patch
    test_car_patch=draw.CarPatch(tar_x,tar_y,tar_yaw,0.0,P.P,color='red')
    test_car_patch.add_to_axes(ax)

#generate refline patch
refline_plot = ax.scatter([], [], s=2)
refline_data=np.zeros((len(R.ref_x),2),float)
for i in range(len(R.ref_x)):
    refline_data[i]=[R.ref_x[i],R.ref_y[i]]
refline_plot.set_offsets(refline_data)
refline_plot.set_color('grey')
refline_plot.set_visible(True)
refline_patch_list.append(refline_plot)
main_refline_plot=ax.scatter([], [], s=2)
main_refline_data=np.zeros((len(R_tar.ref_x),2),float)
for i in range(len(R_tar.ref_x)):
    main_refline_data[i]=[R_tar.ref_x[i],R_tar.ref_y[i]]
main_refline_plot.set_offsets(main_refline_data)
main_refline_plot.set_color('grey')
main_refline_plot.set_visible(True)
refline_patch_list.append(main_refline_plot)

#generate lane boundary patch
lb1_line, = ax.plot(lb1.ref_x, lb1.ref_y, linestyle='-', color='black',linewidth=1)
refline_patch_list.append(lb1_line)
lb2_line, = ax.plot(lb2.ref_x, lb2.ref_y, linestyle='--', color='black',linewidth=1)
refline_patch_list.append(lb2_line)
lb3_line, = ax.plot(lb3.ref_x, lb3.ref_y, linestyle='-', color='black',linewidth=1)
refline_patch_list.append(lb3_line)

def draw_graph(traj,frame_id):
    v_data = []    
    kappa_data = []
    time_data=[]  
    time_now = 0.0

    config_path = '/home/fengmq/code/test_vehicle_planner_kappa_v_limit/proto_data/speed_planner_config.pb.txt'
    config = speed_planner_config_pb2.SpeedPlannerConfig()
    with open(config_path, 'r') as file:
        file_content = file.read()
        res = text_format.Parse(file_content, config)
    delta_t=config.delta_time
    
    for state in traj:  
        v_data.append(state[3])  
        kappa_data.append(state[4])  
        time_data.append(time_now)
        time_now += delta_t  # 注意这里time_now现在只用于计算时间，不直接用于绘图  
    
    # 创建一个1行2列的图形窗口，并返回figure和axes对象  
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))  
    
    # 第一个子图：绘制v随时间变化的折线图
    max_v = max(v_data)
    min_v = min(v_data)
    axs[0].plot(time_data, v_data, label='test vehicle velocity')  
    axs[0].set_title('Velocity')  
    axs[0].set_xlabel('time (s)')  
    axs[0].set_ylabel('velocity (m/s)')
    axs[0].set_ylim(bottom=0 if min_v>0 else min_v*1.5, top=max_v * 2) 
    axs[0].legend()  
    
    # 第二个子图：绘制kappa随时间变化的折线图  
    # from scipy.signal import medfilt1d
    # window_size=3
    # kappa_data = medfilt1d(np.array(kappa_data), kernel_size=window_size)

    max_kappa = max(kappa_data)
    min_kappa = min(kappa_data)
    axs[1].plot(time_data, kappa_data, label='test vehicle curvature')  
    axs[1].set_title('Curvature')  
    axs[1].set_xlabel('time (s)')  
    axs[1].set_ylabel('curvature')  
    axs[1].set_ylim(bottom=0 if min_kappa>0 else min_kappa*1.5, top=max_kappa * 2) 
    axs[1].legend()  
    
    # 调整子图之间的间距  
    plt.tight_layout()  
    
    # # 显示图表  
    # plt.show()
    # import time
    # time.sleep(1)
    # plt.close(fig)
    output_filename = f"frame_{frame_id}.png"
    output_path = os.path.join(output_dir, output_filename)
    print("output_path=",output_path)
    plt.savefig(output_path)
    plt.close(fig)




def init():
    return []


#最后一帧用来清除
def update(frame):
    if frame!=0:
        for i, vehicle in enumerate(test_car_traj_patch_list[frame-1]):
            vehicle.set_visible(False)
        test_car_patch=test_car_patch_list[frame-1]
        test_car_patch.car_poly.set_visible(False)
        test_car_patch.fl_wheel_poly.set_visible(False)
        test_car_patch.fr_wheel_poly.set_visible(False)
        test_car_patch.rl_wheel_poly.set_visible(False)
        test_car_patch.rr_wheel_poly.set_visible(False)
        # main_car_patch=main_car_patch_list[frame-1]
        # main_car_patch.car_poly.set_visible(False)
        # main_car_patch.fl_wheel_poly.set_visible(False)
        # main_car_patch.fr_wheel_poly.set_visible(False)
        # main_car_patch.rl_wheel_poly.set_visible(False)
        # main_car_patch.rr_wheel_poly.set_visible(False)


    if frame==len(test_car_traj_patch_list):
        for i, vehicle in enumerate(test_car_traj_patch_list[frame-1]):
            vehicle.set_visible(False)
        test_car_patch=test_car_patch_list[frame-1]
        test_car_patch.car_poly.set_visible(False)
        test_car_patch.fl_wheel_poly.set_visible(False)
        test_car_patch.fr_wheel_poly.set_visible(False)
        test_car_patch.rl_wheel_poly.set_visible(False)
        test_car_patch.rr_wheel_poly.set_visible(False)
        # main_car_patch=main_car_patch_list[frame-1]
        # main_car_patch.car_poly.set_visible(False)
        # main_car_patch.fl_wheel_poly.set_visible(False)
        # main_car_patch.fr_wheel_poly.set_visible(False)
        # main_car_patch.rl_wheel_poly.set_visible(False)
        # main_car_patch.rr_wheel_poly.set_visible(False)

    else:
        for i, vehicle in enumerate(test_car_traj_patch_list[frame]):
            vehicle.set_visible(True)
        test_car_patch=test_car_patch_list[frame]
        test_car_patch.car_poly.set_visible(True)
        test_car_patch.fl_wheel_poly.set_visible(True)
        test_car_patch.fr_wheel_poly.set_visible(True)
        test_car_patch.rl_wheel_poly.set_visible(True)
        test_car_patch.rr_wheel_poly.set_visible(True)
        # main_car_patch=main_car_patch_list[frame]
        # main_car_patch.car_poly.set_visible(True)
        # main_car_patch.fl_wheel_poly.set_visible(True)
        # main_car_patch.fr_wheel_poly.set_visible(True)
        # main_car_patch.rl_wheel_poly.set_visible(True)
        # main_car_patch.rr_wheel_poly.set_visible(True)
    
    if frame%5==0 or frame<5:
        output_filename = f"animation_{frame}.png"
        output_dir='/home/fengmq/code/test_vehicle_planner_kappa_v_limit/fig'
        output_path = os.path.join(output_dir, output_filename)
        print("output_path=",output_path)
        plt.savefig(output_path,dpi=200)
        draw_graph(test_car_traj_list[frame],frame)

    return 


#删除./fig中的图
import glob
output_dir='/home/fengmq/code/test_vehicle_planner_kappa_v_limit/fig'
png_files = glob.glob(os.path.join(output_dir, '*.png'))  
for file in png_files:  
    try:  
        os.remove(file)  
        print(f"Deleted file: {file}")  
    except OSError as e:  
        print(f"Error: {e.strerror} : {file}")

ani = FuncAnimation(fig, update, frames=len(test_car_traj_patch_list)+1, init_func=init, blit=False,interval=200)

plt.show()