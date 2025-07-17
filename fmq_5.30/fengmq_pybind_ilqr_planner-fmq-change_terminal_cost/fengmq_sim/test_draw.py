import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Polygon

# 假设我们有下列轨迹数据和对应的横摆角(这里只是简单示例)
trajectory = np.array([[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]])  # 轨迹点坐标
yaw_angles = np.pi / 180 * np.array([0, 30, 45, 60, 90])  # 横摆角，转换为弧度

# 假设的多边形障碍物数据
obstacles = [
    np.array([[0.5, 1.5], [1.5, 1.8], [1.2, 2.2], [0.7, 1.7]]),
    np.array([[3.5, 3.5], [4.5, 3.8], [4.2, 4.2], [3.7, 3.7]])
]

# 初始化图表
fig, ax = plt.subplots()
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)

# 绘制障碍物并添加到图表中
for obstacle in obstacles:
    polygon = Polygon(obstacle, closed=True, color='gray', alpha=0.5)
    ax.add_patch(polygon)

# 创建车辆方框并初始化为不可见
vehicles = []
for i in range(len(trajectory)):
    opacity = (i + 1) / len(trajectory)  # 让透明度随着时间增加而增加
    width = 0.2
    height = 0.1
    rect = Rectangle((trajectory[i][0] - width/2, trajectory[i][1] - height/2),
                     width=width, height=height,
                     angle=np.degrees(yaw_angles[i]),  # 使用角度设定旋转
                     color=(0, 0, 1, opacity))  # 使用RGBA定义颜色和透明度
    vehicles.append(rect)
    ax.add_patch(rect)
    rect.set_visible(False)

# 构建动画
def init():
    return []

def update(frame):
    patches = []
    for i, vehicle in enumerate(vehicles):
        if frame >= i:
            vehicle.set_visible(True)
        else:
            vehicle.set_visible(False)
        patches.append(vehicle)
    return patches

ani = FuncAnimation(fig, update, frames=len(trajectory)+1, init_func=init, blit=True)

plt.show()
