import math
import numpy as np
class Map():
    def __init__(self,boundary,sta_obs_key_points):
        #顺序是x_min,x_max,y_min,y_max
        self.boundary=boundary
        self.sta_obs_key_points=sta_obs_key_points
        self.num_of_sta_obs=len(self.sta_obs_key_points)
        self.car_length=3
        self.car_width=1.8



    def ego2world(self,x,y,theta):
        
        pts=[[-self.car_length/2,self.car_width/2],[self.car_length/2,self.car_width/2],
             [self.car_length/2,-self.car_width/2],[-self.car_length/2,-self.car_width/2]]
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


    def check_obs(self,x,y):
        for i in range(len(self.sta_obs_key_points)):
            count=0
            for j in range(0, len(self.sta_obs_key_points[i])):
                s_index = j
                e_index = j + 1
                if j == len(self.sta_obs_key_points[i]) - 1:
                    s_index = j
                    e_index = 0
                if self.is_ray_intersects_segment([x,y], self.sta_obs_key_points[i][s_index],
                                                    self.sta_obs_key_points[i][e_index]):
                    count = count + 1
            if count % 2 == 1:
                return True
        return False



    def check_ego_obs(self,ego_x,ego_y,ego_theta):
        ego_pt_list=self.ego2world(ego_x,ego_y,ego_theta)
        for i in range(len(self.sta_obs_key_points)):
            obs_tar=self.sta_obs_key_points[i]
            for pt in ego_pt_list:
                corner_x=pt[0]
                corner_y=pt[1]
                count=0
                for j in range(0, len(self.sta_obs_key_points[i])):
                    s_index = j
                    e_index = j + 1
                    if j == len(self.sta_obs_key_points[i]) - 1:
                        s_index = j
                        e_index = 0
                    if self.is_ray_intersects_segment([corner_x,corner_y], self.sta_obs_key_points[i][s_index],
                                                    self.sta_obs_key_points[i][e_index]):
                        count = count + 1
                if count % 2 == 1:
                    return True              
            for pt in self.sta_obs_key_points[i]:
                corner_x=pt[0]
                corner_y=pt[1]
                count=0
                for j in range(0, len(ego_pt_list)):
                    s_index = j
                    e_index = j + 1
                    if j == len(ego_pt_list) - 1:
                        s_index = j
                        e_index = 0
                    if self.is_ray_intersects_segment([corner_x,corner_y], ego_pt_list[s_index],
                                                    ego_pt_list[e_index]):
                        count = count + 1
                if count % 2 == 1:
                    return True  
        return False

    def is_ray_intersects_segment(self, point, start, end) -> bool:
        if start[1] == end[1]:  # 排除与射线平行、重合，线段首尾端点重合的情况
            return False
        if start[1] > point[1] and end[1] > point[1]:  # 线段在射线上边
            return False
        if start[1] < point[1] and end[1] < point[1]:  # 线段在射线下边
            return False
        if start[1] == point[1] and end[1] > point[1]:  # 交点为下端点，对应start point
            return False
        if end[1] == point[1] and start[1] > point[1]:  # 交点为下端点，对应end point
            return False
        if start[0] < point[0] and end[1] < point[1]:  # 线段在射线左边
            return False
        seg = end[0] - (end[0] - start[0]) * (end[1] - point[1]) / (end[1] - start[1])  # 求交
        if seg < point[0]:
            return False
        return True

    def point_to_line(self, px, py, x1, y1, x2, y2):
        line_magnitude = self.point_to_point(x1, y1, x2, y2)
        if line_magnitude < 0.00000001:
            # 如果线段距离很近，直接返回P到A的距离
            return self.point_to_point(px, py, x1, y1)
        u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))  # 向量AB·向量AP
        u = u1 / (line_magnitude * line_magnitude)  # 向量AP在向量AB方向的投影与向量AB模的比值
        if (u < 0) or (u > 1):
            # 点到直线的投影不在线段内, 计算点到两个端点距离的最小值即为"点到线段最小距离"
            return min(self.point_to_point(px, py, x1, y1), self.point_to_point(px, py, x2, y2))
        # 投影点在线段内部, 计算方式同点到直线距离, u 为投影点距离x1在x1x2上的比例, 以此计算出投影点的坐标
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        return self.point_to_point(px, py, ix, iy)

    def point_to_point(self, x1, y1, x2, y2):
        lineMagnitude = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
        return lineMagnitude


    # def check_obs(self,grid_str):
    #     #print(grid_str)
    #     if grid_str in self.obstacles:
    #         if self.obstacles[grid_str]==1:
    #             return 1
    #     return 0
    #
    # def str_to_location(self,str):
    #     x=""
    #     y=""
    #     douhao=0
    #     for i in range(0,len(str)):
    #         if str[i]==',':
    #             douhao=1
    #             continue
    #         if douhao==0:
    #             x+=str[i]
    #         elif douhao==1:
    #             y+=str[i]
    #     x=float(x)*self.resolution
    #     y=float(y)*self.resolution
    #     return x,y
    #
    def is_out(self,x,y):
        if x<self.boundary[0] or x>self.boundary[1] or y<self.boundary[2] or y>self.boundary[3]:
            return 1
        return 0


