import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Polygon
sys.path.append('/home/fengmq/code/fengmq_pybind_ilqr_planner/build')
sys.path.append('/home/fengmq/code/fengmq_pybind_ilqr_planner/build/proto')
sys.path.append('/home/fengmq/code/fengmq_pybind_ilqr_planner/global_planner')
import pybind_test
pybind_test.TestPybind(3.14)
print("success import pybind")
import ilqr_config_pb2
import ilqr_data_pb2
import plan_common_pb2
import virtual_lane_smooth_pb2
print("success import proto")
from google.protobuf import text_format
import global_planner 
import map as Map
import csv
import math

#python3 ../fengmq_sim/fengmq_sim.py >eval.log 2>&1

class FengmqPlanerSim():
    def __init__(self):
        self.start_state=plan_common_pb2.State()
        self.tar_state=plan_common_pb2.State()
        choose_map_idx=8
        csv_file_path = '/home/fengmq/code/fengmq_pybind_ilqr_planner/fengmq_sim/map.csv'  
        row_idx=0
        obstacles={}
        with open(csv_file_path, mode='r', newline='') as csv_file:  
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:  
                if row_idx==choose_map_idx:
                    map_min_x=float(row[0])
                    map_max_x=float(row[1])
                    map_min_y=float(row[2])
                    map_max_y=float(row[3])
                    self.start_state.x=float(row[4])
                    self.start_state.y=float(row[5])
                    self.start_state.theta=float(row[6])
                    self.tar_state.x=float(row[7])
                    self.tar_state.y=float(row[8])
                    self.tar_state.theta=float(row[9])
                    obs_temp=[]
                    obs_num=0
                    col_idx=11
                    while col_idx<len(row):
                        if row[col_idx]=='obs':
                            obstacles[obs_num]=obs_temp
                            obs_temp=[]
                            obs_num+=1
                            col_idx+=1
                        else:
                            obs_temp.append([float(row[col_idx]),float(row[col_idx+1])])
                            col_idx+=2
                    self.map=self.CreateMap(map_min_x,map_max_x,map_min_y,map_max_y,obstacles)
                    break
                else:
                    row_idx+=1


        #第一帧自车位置在起点
        self.ego_state=self.start_state



    def RunHybridAStar(self,start_state,tar_state):
        start_state=[start_state.x,start_state.y,start_state.theta]
        end_state=[tar_state.x,tar_state.y,tar_state.theta]       
        x_list,y_list,theta_list, gear = self.global_planner.plan(start_state,end_state)
        # import pdb;pdb.set_trace()
        x_list_new=[x_list[0]]
        y_list_new=[y_list[0]]
        theta_list_new=[theta_list[0]]
        gear_new=[gear[0]]
        for i in range(1,len(x_list)):
            if x_list[i]==x_list[i-1] and y_list[i]==y_list[i-1]:
                continue
            x_list_new.append(x_list[i])
            y_list_new.append(y_list[i])
            theta_list_new.append(theta_list[i])
            gear_new.append(gear[i])

        self.global_path = np.array([x_list_new,y_list_new,theta_list_new,gear_new]).T
        #存的是换档前的idx
        change_gear_idx=[-1]
        for i in range(1,len(self.global_path)):
            if self.global_path[i,3]!=self.global_path[i-1,3]:
                change_gear_idx.append(i-1)
        change_gear_idx.append(len(self.global_path)-1)
        print("change_gear_idx=",change_gear_idx)
        self.change_gear_idx=change_gear_idx
            


    def CreateMap(self,map_min_x,map_max_x,map_min_y,map_max_y,obstacles):
        boundary=[map_min_x,map_max_x,map_min_y,map_max_y]
        map=Map.Map(boundary,obstacles)
        return map


    def LoadParams(self):
        ilqr_params_path = '/home/fengmq/code/fengmq_pybind_ilqr_planner/proto_data/ilqr_params.pb.txt'
        ilqr_params = virtual_lane_smooth_pb2.IlqrParams()
        with open(ilqr_params_path, 'r') as file:
            file_content = file.read()
            res = text_format.Parse(file_content, ilqr_params)
        print("success load parmas") 

        hybrid_astar_params_path = '/home/fengmq/code/fengmq_pybind_ilqr_planner/proto_data/hybrid_astar_params.pb.txt'
        hybrid_astar_params = virtual_lane_smooth_pb2.HybridAStarParams()
        with open(hybrid_astar_params_path, 'r') as file:
            file_content = file.read()
            res = text_format.Parse(file_content, hybrid_astar_params)
        print("success load parmas") 

        return ilqr_params,hybrid_astar_params
    
    def NormalizeAngle(self,angle):
        while angle > math.pi:
            angle-=2*math.pi
        while angle < -math.pi:
            angle+=2*math.pi   
        return angle         

    
    def GenerateIlqrProtoMsg(self):
        ilqr_proto_msg=virtual_lane_smooth_pb2.IlqrProtoMsg()
        closest_idx=0
        min_dis_2=float('inf')
        for i,raw_point in enumerate(self.global_path):
            temp_dis_2=(raw_point[0]-self.ego_state.x)**2+(raw_point[1]-self.ego_state.y)**2
            if temp_dis_2<min_dis_2:
                min_dis_2=temp_dis_2
                closest_idx=i

        self.ego_gear=self.global_path[closest_idx,3]
        # import pdb;pdb.set_trace()

        print("global path = ",self.global_path)
        raw_pt_start_idx=None
        raw_pt_end_idx=None
        # import pdb;pdb.set_trace()
        for i in range(1,len(self.change_gear_idx)):
            if closest_idx>self.change_gear_idx[i-1] and closest_idx<=self.change_gear_idx[i]:
                raw_pt_start_idx=self.change_gear_idx[i-1]
                raw_pt_end_idx=self.change_gear_idx[i]
        # import pdb;pdb.set_trace()
        print("self.ego_gear=",self.ego_gear)
        for i in range(raw_pt_start_idx,raw_pt_end_idx+1):
            raw_pt=virtual_lane_smooth_pb2.RawPoint()
            raw_pt.x=self.global_path[i,0]
            raw_pt.y=self.global_path[i,1]
            if self.ego_gear==-1 or self.ego_gear==-2:
                raw_pt.theta=self.NormalizeAngle(self.global_path[i,2]+math.pi)
            else:
                raw_pt.theta=self.global_path[i,2]
            ilqr_proto_msg.raw_points.add().CopyFrom(raw_pt)

        if self.ego_gear==-1 or self.ego_gear==-2:
            self.ego_state.theta=self.NormalizeAngle(self.ego_state.theta+math.pi)
            self.ego_state.curvature=-self.ego_state.curvature

        ilqr_proto_msg.ego_state.CopyFrom(self.ego_state)

        if self.ego_gear==-1 or self.ego_gear==-2:
            self.ego_state.theta=self.NormalizeAngle(self.ego_state.theta+math.pi)
            self.ego_state.curvature=-self.ego_state.curvature

        for i in range(len(self.map.sta_obs_key_points)):
            polygen=plan_common_pb2.Polygen()
            for pt in self.map.sta_obs_key_points[i]:
                pt2d=plan_common_pb2.Point2d()
                pt2d.x=pt[0]
                pt2d.y=pt[1]
                polygen.pts.add().CopyFrom(pt2d)
            print("the",i,"polygen put in proto")
            ilqr_proto_msg.polygens.add().CopyFrom(polygen)

        return ilqr_proto_msg


    def Process(self):
        self.ilqr_params,self.hybrid_astar_params=self.LoadParams()
        #初始化全局规划器
        self.global_planner = global_planner.Global_planner_hybrid_astar(self.map,self.hybrid_astar_params)
        #run hybrid a star
        self.RunHybridAStar(self.start_state,self.tar_state)
        for i,pt in enumerate(self.global_path):
            print("pt=","x:",pt[0],"y:",pt[1],"theta:",pt[2],"gear",pt[3])
        proto_msg_result_list=[]
        self.frame_interval=5
        step_num=int(len(self.global_path)/self.frame_interval-1)
        global_path_idx=0
        pre_global_path_idx=-1


        for i in range(0,step_num):
            ilqr_proto_msg=self.GenerateIlqrProtoMsg()
            # import pdb;pdb.set_trace()
            proto_msg_result_string=pybind_test.RunIlqrOptimizer(ilqr_proto_msg.SerializeToString(),self.ilqr_params.SerializeToString())
            proto_msg_result=virtual_lane_smooth_pb2.IlqrProtoMsg()
            proto_msg_result.ParseFromString(proto_msg_result_string)
            print("proto_msg_result_iteration num=",len(proto_msg_result.ilqr_iteration_datas))
            if len(proto_msg_result.ilqr_iteration_datas)==0:
                continue
            proto_msg_result_list.append(proto_msg_result)
            print("len of proto_msg_result.ilqr_iteration_datas=",len(proto_msg_result.ilqr_iteration_datas))
            self.SetEgoState(pre_global_path_idx,global_path_idx,proto_msg_result.ilqr_iteration_datas[len(proto_msg_result.ilqr_iteration_datas)-1])
            pre_global_path_idx,global_path_idx=self.UpdateGlobalIdx(global_path_idx)

        self.draw_graph(proto_msg_result_list,-1)
        self.draw(proto_msg_result_list)


    def UpdateGlobalIdx(self,pre_idx):
        idx=pre_idx+self.frame_interval
        return pre_idx,idx

    def SetEgoState(self,pre_global_path_idx,global_path_idx,traj):
        #先检查idx是否跨过换档点
        is_change_gear=False
        for idx in self.change_gear_idx:
            if global_path_idx>=idx and pre_global_path_idx<=idx:
                is_change_gear=True
                break

        is_close_to_gear_change=False
        new_idx=-1
        for idx in self.change_gear_idx:
            if global_path_idx<=idx and abs(global_path_idx-idx)<4:
                is_close_to_gear_change=True
                new_idx=idx+1
                break
        # import pdb;pdb.set_trace()
        if is_change_gear:
            self.ego_state.x=self.global_path[global_path_idx][0]
            self.ego_state.y=self.global_path[global_path_idx][1]
            self.ego_state.theta=self.global_path[global_path_idx][2]           

        elif is_close_to_gear_change:
            self.ego_state.x=self.global_path[new_idx][0]
            self.ego_state.y=self.global_path[new_idx][1]
            self.ego_state.theta=self.global_path[new_idx][2]  

        else:
            min_dis2=float('inf')
            min_dis2_idx=-1
            for i,traj_pt in enumerate(traj.states):
                temp_dis2=(traj_pt.x-self.global_path[global_path_idx][0])**2+(traj_pt.y-self.global_path[global_path_idx][1])**2
                if temp_dis2<min_dis2:
                    min_dis2=temp_dis2
                    min_dis2_idx=i
            
            if self.ego_gear==-1 or self.ego_gear==-2:
                self.ego_state.x=traj.states[min_dis2_idx].x
                self.ego_state.y=traj.states[min_dis2_idx].y
                self.ego_state.theta=self.NormalizeAngle(traj.states[min_dis2_idx].theta+math.pi)
                self.ego_state.curvature=-traj.states[min_dis2_idx].kappa
                self.ego_state.v=traj.states[min_dis2_idx].v
            else:
                self.ego_state.x=traj.states[min_dis2_idx].x
                self.ego_state.y=traj.states[min_dis2_idx].y
                self.ego_state.theta=traj.states[min_dis2_idx].theta
                self.ego_state.curvature=traj.states[min_dis2_idx].kappa
                self.ego_state.v=traj.states[min_dis2_idx].v

    


    def cal_vehicle_pts(self,x,y,theta,width,long):
        
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
    

    def draw(self,proto_msg_result_list):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        # ax.set_xlim(self.map.boundary[0], self.map.boundary[1])
        # ax.set_ylim(self.map.boundary[2], self.map.boundary[3])
        view_edge=15
        # ax.set_xlim(min(self.global_path[:,0])-view_edge, max(self.global_path[:,0])+view_edge)
        # ax.set_ylim(min(self.global_path[:,1])-20, max(self.global_path[:,1])+5)
        ax.set_xlim(16,21)
        ax.set_ylim(39.5,41)
        traj_pt_lists=[]
        warm_start_plot_list=[]
        obs_polygen_list=[]
        rectangle_list=[]
        for proto_msg_result in proto_msg_result_list:
            traj_pt_list=[]
            traj=proto_msg_result.ilqr_iteration_datas[len(proto_msg_result.ilqr_iteration_datas)-1]
            for i,traj_pt in enumerate(traj.states):
                opacity = (i + 1) / len(traj.states)  # 让透明度随着时间增加而增加
                width = 1
                long = 2
                print("traj_theta=",traj_pt.theta)
                vehicle_pts=self.cal_vehicle_pts(traj_pt.x , traj_pt.y, traj_pt.theta,width,long)
                rect = Polygon(vehicle_pts,
                                closed=True, color='blue', alpha=1-opacity)  
                traj_pt_list.append(rect)
                ax.add_patch(rect)
                rect.set_visible(False)
            traj_pt_lists.append(traj_pt_list)

        for proto_msg_result in proto_msg_result_list:
            warm_start_plot = ax.scatter([], [], s=5)
            warm_start_data=np.zeros((len(proto_msg_result.warm_start_data.states),2),float)
            for k,warm_start_pt in enumerate(proto_msg_result.warm_start_data.states):
                warm_start_data[k]=[warm_start_pt.x,warm_start_pt.y]
            # print("warm_start_data=",warm_start_data)
            warm_start_plot.set_offsets(warm_start_data)
            warm_start_plot.set_color('red')
            warm_start_plot.set_visible(False)
            warm_start_plot_list.append(warm_start_plot)

        for i in range(len(self.map.sta_obs_key_points)):
            obs_pt_list=[]
            for pt in self.map.sta_obs_key_points[i]:
                obs_pt_list.append((pt[0],pt[1]))
            poly = Polygon(obs_pt_list, closed=True, color='black', alpha=0.5)
            ax.add_patch(poly)

        for proto_msg_result in proto_msg_result_list:
            rectangle_one_frame=[]
            for i,rectangle in enumerate(proto_msg_result.rectangles):
                if i%5==0:
                    rec_pt_list=[]
                    for pt in rectangle.pts:
                        print("rectangle pt=[",pt.x,",",pt.y,"]")
                        rec_pt_list.append((pt.x,pt.y))
                    poly = Polygon(rec_pt_list, closed=True,fill=False, color='orange', alpha=1)
                    poly.set_visible(False)
                    rectangle_one_frame.append(poly)
                    ax.add_patch(poly)
            rectangle_list.append(rectangle_one_frame)

        hybrid_astar_plot = ax.scatter([], [], s=8)
        hybrid_astar_data=np.zeros((len(self.global_path),2),float)
        for k,pt in enumerate(self.global_path):
            hybrid_astar_data[k]=[pt[0],pt[1]]
        hybrid_astar_plot.set_offsets(hybrid_astar_data)
        hybrid_astar_plot.set_color('green')

        def init():
            return []


        #最后一帧用来清除
        def update(frame):
            if frame!=0:
                for i, vehicle in enumerate(traj_pt_lists[frame-1]):
                    vehicle.set_visible(False)
                warm_start_plot_list[frame-1].set_visible(False)
                for rectangle in rectangle_list[frame-1]:
                    rectangle.set_visible(False)

            if frame==len(traj_pt_lists):
                for i, vehicle in enumerate(traj_pt_lists[frame-1]):
                    vehicle.set_visible(False)
                warm_start_plot_list[frame-1].set_visible(False)
                for rectangle in rectangle_list[frame-1]:
                    rectangle.set_visible(False)
            else:
                for i, vehicle in enumerate(traj_pt_lists[frame]):
                    vehicle.set_visible(True)
                warm_start_plot_list[frame].set_visible(True)
                for rectangle in rectangle_list[frame]:
                    rectangle.set_visible(True)

            output_filename = f"animation_{frame}.png"
            output_dir='/home/fengmq/code/fengmq_pybind_ilqr_planner/fig/'
            output_path = os.path.join(output_dir, output_filename)
            print("output_path=",output_path)
            plt.savefig(output_path,dpi=700)
            return 

        ani = FuncAnimation(fig, update, frames=len(traj_pt_lists)+1, init_func=init, blit=False,interval=300)

        plt.show()

    def draw_graph(self, proto_msg_result_list, frame_id):
        import glob
        output_dir='/home/fengmq/code/fengmq_pybind_ilqr_planner/fig/'
        png_files = glob.glob(os.path.join(output_dir, '*.png'))  
        for file in png_files:  

            try:  

                os.remove(file)  

                print(f"Deleted file: {file}")  

            except OSError as e:  

                print(f"Error: {e.strerror} : {file}")
        for i, proto_msg_result in enumerate(proto_msg_result_list):  
            if frame_id != -1 and i != frame_id:  
                continue  
            
            v_data = []    
            kappa_data = []
            time_data=[]  
            traj = proto_msg_result.ilqr_iteration_datas[len(proto_msg_result.ilqr_iteration_datas)-1]  
            time_now = 0.0  
            
            for state in traj.states:  
                v_data.append(state.v)  
                kappa_data.append(state.kappa)  
                time_data.append(time_now)
                time_now += self.ilqr_params.delta_t  # 注意这里time_now现在只用于计算时间，不直接用于绘图  
            
            # 创建一个1行2列的图形窗口，并返回figure和axes对象  
            fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))  
            
            # 第一个子图：绘制v随时间变化的折线图
            max_v = max(v_data)
            min_v = min(v_data)
            axs[0].plot(time_data, v_data, label='velocity')  
            axs[0].set_title('Velocity')  
            axs[0].set_xlabel('time (s)')  
            axs[0].set_ylabel('velocity (m/s)')
            axs[0].set_ylim(bottom=0 if min_v>0 else min_v*1.5, top=max_v * 2) 
            axs[0].legend()  
            
            # 第二个子图：绘制kappa随时间变化的折线图  
            max_kappa = max(kappa_data)
            min_kappa = min(kappa_data)
            axs[1].plot(time_data, kappa_data, label='curvature')  
            axs[1].set_title('Curvature')  
            axs[1].set_xlabel('time (s)')  
            axs[1].set_ylabel('curvature')  
            axs[1].set_ylim(bottom=0 if min_kappa>0 else min_kappa*1.5, top=0 if max_kappa<0 else max_kappa*1.5) 
            axs[1].legend()  
            
            # 调整子图之间的间距  
            plt.tight_layout()      
            
            # # 显示图表  
            # plt.show()
            # import time
            # time.sleep(1)
            # plt.close(fig)
            output_filename = f"frame_{i}.png"
            output_path = os.path.join(output_dir, output_filename)
            print("output_path=",output_path)
            plt.savefig(output_path,dpi=500)
            plt.close(fig)


if __name__ == "__main__":
    fengmq_sim=FengmqPlanerSim()
    fengmq_sim.Process()