import os
import matplotlib.pyplot as plt
import numpy as np

# 获取"data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/data.txt")

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

# 保存图片到当前文件所在路径
filename = "path_2D.png"
plt.savefig(os.path.join(os.getcwd(), filename))

# 显示图形
plt.show()
