import numpy as np
import matplotlib.pyplot as plt
x_arr = []
y_arr = []
bias_x = 0.0
bias_y = 0.0
loc_x = 0.0
loc_y = 0.0
heading = 0.0
log_path = "/home/wjh/code/unitree_ros2/wjh.log"
with open(log_path, 'r') as log_file:
    while True:
        glog_line = log_file.readline()
        if "bias_x" in glog_line:
            bias_x = glog_line.split("bias_x:")[-1].split(" ")[0]
            bias_x = float(bias_x)
        if "bias_y" in glog_line:
            bias_y = glog_line.split("bias_y:")[-1].split(" ")[0]
            bias_y = float(bias_y)
        if "loc_x" in glog_line:
            loc_x = glog_line.split("loc_x:")[-1].split(" ")[0]
            loc_x = float(loc_x)
        if "loc_y" in glog_line:
            loc_y = glog_line.split("loc_y:")[-1].split(" ")[0]
            loc_y = float(loc_y)
        if "heading" in glog_line:
            heading = glog_line.split("heading:")[-1].split(" ")[0]
            heading = float(heading)
        if "boundary_map_x:" in glog_line:
            x = glog_line.split("boundary_map_x:")[-1].split(" ")[0]
            x_arr.append(float(x))
        if "boundary_map_y:" in glog_line:
            y = glog_line.split("boundary_map_y:")[-1].split(" ")[0]
            y_arr.append(float(y))
        if "end_record" in glog_line:
            break
        if glog_line=="\n" or glog_line=="":
            break
plt.scatter(x_arr, y_arr)
plt.scatter(loc_x, loc_y)
plt.scatter(bias_x, bias_y)
# 计算x和y的分量，这里假设箭头长度为1
x_component = np.cos(heading)
y_component = np.sin(heading)

plt.annotate("", xy=(x_component, y_component), xytext=(loc_x, loc_y),
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color='blue'))

plt.show()
