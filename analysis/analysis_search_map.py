import numpy as np
import matplotlib.pyplot as plt
x_arr = []
y_arr = []
left_x_arr = []
left_y_arr = []
right_x_arr = []
right_y_arr = []
search_x_arr = []
search_y_arr = []
traj_x_arr = []
traj_y_arr = []
traj_vx_arr = []
traj_vy_arr = []
traj_s_arr = []
traj_t_arr = []
traj_yaw_arr = []
traj_vyaw_arr = []
end_x = 0.0
end_y = 0.0
bias_x = 0.0
bias_y = 0.0
loc_x = 0.0
loc_y = 0.0
heading = 0.0
log_path = "/home/wjh/code/unitree_ros2/wjh.log"
with open(log_path, 'r') as log_file:
    while True:
        glog_line = log_file.readline()
        if "end_x" in glog_line:
            end_x = glog_line.split("end_x:")[-1].split(" ")[0]
            end_x = float(end_x)
        if "end_y" in glog_line:
            end_y = glog_line.split("end_y:")[-1].split(" ")[0]
            end_y = float(end_y)
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
        if "left_x:" in glog_line:
            x = glog_line.split("left_x:")[-1].split(" ")[0]
            left_x_arr.append(float(x))
        if "left_y:" in glog_line:
            y = glog_line.split("left_y:")[-1].split(" ")[0]
            left_y_arr.append(float(y))
        if "right_x:" in glog_line:
            x = glog_line.split("right_x:")[-1].split(" ")[0]
            right_x_arr.append(float(x))
        if "right_y:" in glog_line:
            y = glog_line.split("right_y:")[-1].split(" ")[0]
            right_y_arr.append(float(y))
        if "search_x:" in glog_line:
            x = glog_line.split("search_x:")[-1].split(" ")[0]
            search_x_arr.append(float(x))
        if "search_y:" in glog_line:
            y = glog_line.split("search_y:")[-1].split(" ")[0]
            search_y_arr.append(float(y))
        if "trajectory_x:" in glog_line:
            x = glog_line.split("trajectory_x:")[-1].split(" ")[0]
            traj_x_arr.append(float(x))
        if "trajectory_y:" in glog_line:
            y = glog_line.split("trajectory_y:")[-1].split(" ")[0]
            traj_y_arr.append(float(y))
        if "trajectory_yaw:" in glog_line:
            y = glog_line.split("trajectory_yaw:")[-1].split(" ")[0]
            traj_yaw_arr.append(float(y))
        if "trajectory_vyaw:" in glog_line:
            y = glog_line.split("trajectory_vyaw:")[-1].split(" ")[0]
            traj_vyaw_arr.append(float(y))
        if "trajectory_vx:" in glog_line:
            x = glog_line.split("trajectory_vx:")[-1].split(" ")[0]
            traj_vx_arr.append(float(x))
        if "trajectory_vy:" in glog_line:
            y = glog_line.split("trajectory_vy:")[-1].split(" ")[0]
            traj_vy_arr.append(float(y))
        if "trajectory_s:" in glog_line:
            y = glog_line.split("trajectory_s:")[-1].split(" ")[0]
            traj_s_arr.append(float(y))
        if "trajectory_t:" in glog_line:
            y = glog_line.split("trajectory_t:")[-1].split(" ")[0]
            traj_t_arr.append(float(y))
        if "end_record" in glog_line:
            break
        if glog_line=="\n" or glog_line=="":
            break
plt.scatter(search_x_arr, search_y_arr)
plt.scatter(x_arr, y_arr)
plt.scatter(end_x, end_y)
plt.scatter(left_x_arr, left_y_arr)
plt.scatter(right_x_arr, right_y_arr)
plt.scatter(loc_x, loc_y)
plt.scatter(bias_x, bias_y)
# 计算x和y的分量，这里假设箭头长度为1
x_component = loc_x + np.cos(heading) * 6
y_component = loc_y + np.sin(heading) * 6

plt.annotate("", xy=(x_component, y_component), xytext=(loc_x, loc_y),
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color='blue'))
plt.figure()
plt.scatter(search_x_arr, search_y_arr,label='search')
plt.scatter(traj_x_arr, traj_y_arr,label='trajectory')
plt.legend()
plt.figure()
plt.scatter(traj_t_arr, traj_s_arr,label='s-t')
plt.legend()
plt.figure()
plt.scatter(traj_t_arr, traj_yaw_arr,label='s-yaw')
plt.scatter(traj_t_arr, traj_vyaw_arr,label='s-vyaw')
plt.legend()
plt.figure()
plt.scatter(traj_t_arr, traj_vx_arr,label='s-vx')
plt.scatter(traj_t_arr, traj_vy_arr,label='s-vy')
plt.legend()
plt.axis('equal')
plt.show()
