import os
import matplotlib.pyplot as plt
import numpy as np

# 获取"data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/rrt_data.txt")

# 从data.txt文件中读取数据点
points = []
with open(data_path, "r") as f:
    for line in f:
        x, y, z = [float(x) for x in line.strip().split(",")]
        points.append((x, y, z))

# 绘制每个二维点
fig, ax = plt.subplots()
for i in range(len(points)):
    x, y, _ = points[i]
    ax.scatter(x, y)
    ax.set_aspect(1)
    # ax.text(x, y, f"({x:.2f}, {y:.2f})", fontsize=8)

# 绘制圆形障碍物
obstacles = [((5, 5), 2.0), ((7, 7), 1.0)]
for obstacle in obstacles:
    center, radius = obstacle
    circle = plt.Circle(center, radius, color='r', fill=False)
    ax.add_artist(circle)

# 连接所有的二维点
for i in range(len(points)-1):
    x1, y1, _ = points[i]
    x2, y2, _ = points[i+1]
    ax.plot([x1, x2], [y1, y2])

# 获取"data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/mpc_data.txt")

# 从文件中读取位姿数据
poses = []
with open(data_path, "r") as f:
    for line in f:
        x, y, yaw = line.strip().split()
        poses.append((float(x), float(y), float(yaw)))

# 将每个位姿绘制为空心点和实线段

# 绘制起点和终点
start = poses[0]
end = poses[-1]
ax.plot(start[0], start[1], 'go', markersize=10)
ax.arrow(start[0], start[1], np.cos(start[2])/10, np.sin(start[2])/10,
         head_width=0.02, head_length=0.09, fc='g', ec='g')
ax.plot(end[0], end[1], 'bo', markersize=10)
ax.arrow(end[0], end[1], np.cos(end[2])/10, np.sin(end[2])/10,
         head_width=0.02, head_length=0.09, fc='b', ec='b')

# 绘制每个位姿的空心点和实线段
for i in range(0, len(poses)-10, 10):
    x1, y1, yaw1 = poses[i]
    x2, y2, yaw2 = poses[i+10]
    ax.plot([x1, x2], [y1, y2], 'r')
    ax.plot(x1, y1, 'wo', mec='r', mew=2)

# # 保存图片到当前文件所在路径
# filename = "mpc_path.png"
# plt.savefig(os.path.join(os.getcwd(), filename))

plt.show()
