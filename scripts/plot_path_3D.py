import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D, axes3d

import numpy as np

# 获取"data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/rrt_data.txt")

# 从data.txt文件中读取数据点
points = []
with open(data_path, "r") as f:
    for line in f:
        x, y, z = [float(x) for x in line.strip().split(",")]
        points.append((x, y, z))

# 创建一个三维坐标系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制每个三维点
for i in range(len(points)):
    x, y, z = points[i]
    ax.scatter(x, y, z)
    ax.text(x, y, z, f"({x:.2f}, {y:.2f}, {z:.2f})", fontsize=8)


# 绘制球形障碍物
obstacles = [((5, 5, 0), 2.0), ((7, 7, 0), 1.0)]
for obstacle in obstacles:
    center, radius = obstacle
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    ax.plot_surface(x, y, z, rstride=1, cstride=1, color='r')


# 连接所有的三维点
for i in range(len(points)-1):
    x1, y1, z1 = points[i]
    x2, y2, z2 = points[i+1]
    ax.plot([x1, x2], [y1, y2], [z1, z2])

# 保存图片到当前文件所在路径
filename = "path_3D.png"
plt.savefig(os.path.join(os.getcwd(), filename))

# 显示图形
plt.show()
