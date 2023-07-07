import os
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams

# 设置全局字体大小
rcParams.update({'font.size': 14})

fig, ax = plt.subplots()

# 绘制RRT结果
# 获取"rrt_data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/rrt_data.txt")
with open(data_path, 'r') as f:
    data = f.readlines()

# 解析数据
x0 = []
y0 = []
yaw0 = []
for line in data:
    line = line.strip().split(',')
    x0.append(float(line[0]))
    y0.append(float(line[1]))
    yaw0.append(float(line[2]))
plt.plot(x0, y0, ':r', label='RRT')

points = []
with open(data_path, "r") as f:
    for line in f:
        x, y, z = [float(x) for x in line.strip().split(",")]
        points.append((x, y, z))

# 绘制每个二维点
for i in range(len(points)):
    x, y, _ = points[i]
    ax.scatter(x, y)
    ax.set_aspect(1)
    # ax.text(x, y, f"({x:.2f}, {y:.2f})", fontsize=8)

# 绘制圆形障碍物
obstacles = [((5, 5), 2.0), ((7, 7), 1.0)]
for obstacle in obstacles:
    center, radius = obstacle
    circle = plt.Circle(center, radius, color='k',
                        fill=False, label='OBSTACLE')
    ax.add_artist(circle)

# 绘制优化后的轨迹
# 获取"opt_path.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/opt_path.txt")
with open(data_path, 'r') as f:
    data = f.readlines()

# 解析数据
x1 = []
y1 = []
yaw1 = []
time = []
for line in data:
    line = line.strip().split()
    time.append(float(line[0]))
    x1.append(float(line[1]))
    y1.append(float(line[2]))
    yaw1.append(float(line[3]))

plt.plot(x1, y1, 'g', label='OPT')

# 绘制MPC结果
# 获取"mpc_data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/mpc_data.txt")
with open(data_path, "r") as f:
    data = f.readlines()

# 解析数据
x2 = []
y2 = []
yaw2 = []
for line in data:
    line = line.strip().split()
    x2.append(float(line[0]))
    y2.append(float(line[1]))
    yaw2.append(float(line[2]))

plt.plot(x2, y2, '--b', label='MPC')

# # 保存图片到当前文件所在路径
# filename = "mpc_path.png"
# plt.savefig(os.path.join(os.getcwd(), filename))

# 添加图例
plt.legend()

# 显示图形
plt.show()
