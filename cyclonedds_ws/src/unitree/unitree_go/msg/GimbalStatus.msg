# GimbalStatus.msg

float64 yaw       # 云台的横摆角度
float64 pitch     # 云台的俯仰角度
float64 roll      # 云台的滚动角度
float64 max_temp  # 云台温度
float64 min_temp  # 云台温度
builtin_interfaces/Time angle_timestamp  # 消息更新时间
builtin_interfaces/Time temp_timestamp  # 消息更新时间
