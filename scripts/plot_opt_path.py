import os
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置全局字体大小
rcParams.update({'font.size': 14})

# 获取"data.txt"文件的路径
data_path = os.path.join(os.path.dirname(__file__), "../data/opt_path.txt")

# 读取数据文件
with open(data_path, 'r') as f:
    data = f.readlines()

# 解析数据
x = []
y = []
z = []
vx = []
vy = []
vz = []
time = []
for line in data:
    line = line.strip().split()
    time.append(float(line[0]))
    x.append(float(line[1]))
    y.append(float(line[2]))
    z.append(float(line[3]))
    vx.append(float(line[4]))
    vy.append(float(line[5]))
    vz.append(float(line[6]))

# 创建子图1
plt.subplot(3, 1, 1)
plt.plot(time, x, 'b', label='X Position')
plt.plot(time, vx, 'r', label='X Velocity')
plt.ylabel('Position/Velocity')
plt.legend()

# 创建子图2
plt.subplot(3, 1, 2)
plt.plot(time, y, 'b', label='Y Position')
plt.plot(time, vy, 'r', label='Y Velocity')
plt.ylabel('Position/Velocity')
plt.legend()

# 创建子图3
plt.subplot(3, 1, 3)
plt.plot(time, z, 'b', label='Z Position')
plt.plot(time, vz, 'r', label='Z Velocity')
plt.xlabel('Time')
plt.ylabel('Position/Velocity')
plt.legend()

# 显示图形
plt.show()
