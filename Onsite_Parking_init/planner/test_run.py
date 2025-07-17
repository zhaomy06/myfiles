import json
import input.make_map as mp
from planner.hybridastar import planner as planner
from input import make_car
from utils import replay
from utils import map_display as mdp
import matplotlib.pyplot as plt


def main():
    # 输入input文件夹下场景文件
    map_path = '../input/B03.json'  # 读取静态地图
    map_path_dyobs = '../input/B03_dyobs.json'  # 读取动态地图
    # mdp.map_display(map_path) #  仅绘制地图

    ox, oy, sp, gp = mp.make_map(map_path)
    # gp为所有停车位信息，包括车位中心点的横坐标、纵坐标，朝向，车位四个顶点坐标，车位出口横坐标，车位出口纵坐标，以及从入口到该停车位的标准距离信息
    sx, sy, syaw0 = sp['x'], sp['y'], sp['yaw']  # sp为泊车起点位置，由车辆起点横坐标x，纵坐标y，偏航角yaw三个属性组成
    C = make_car.C

    # 获取动态障碍物
    dyobs_x, dyobs_y, dyobs_yaw = mp.get_obs(map_path_dyobs)

    # 获取目标停车位
    park = '3'
    gx, gy, gyaw0 = gp[park]['x_end'], gp[park]['y_end'], gp[park]['yaw']

    # 规划算法
    path = planner.hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0, ox, oy, C.XY_RESO, C.YAW_RESO, dyobs_x, dyobs_y,
                                         dyobs_yaw)
    # 算法测试结果保存
    if not path:
        print("Searching failed!")
        return
    output_dit = {
        "parking": park,
        "output_x": path.x,
        "output_y": path.y,
        "output_yaw": path.yaw,
        "output_dir": path.direction,
    }  # with open('../output/result_B03_3.json', "w") as file
    with open(f"../output/result_{map_path.split('/')[-1].split('.json')[0]}_{park}.json", "w") as file:
        json.dump(output_dit, file)  # 将 output_dit 字典以 JSON 格式写入文件

    # 仿真回放   result_path = '../output/result_B03_3.json'
    result_path = f"../output/result_{map_path.split('/')[-1].split('.json')[0]}_{park}.json"
    replay.replay(map_path, result_path, map_path_dyobs)


if __name__ == '__main__':
    main()
