
std_msgs/Header header

# 控制模式
# 0: TrajectoryFollow # 轨迹跟踪，需要设置[t,x,y,yaw,vx,vy,vyaw]
# 1: SpeedLevel       # 速度仅可设置档位-1,0,1
# 2: BodyHeight       # 与默认身高相对指[-0.18~0.03](m)
# 3: Damp             # 无参数
# 4: BalanceStand     # 无参数
# 5: StopMove         # 无参数
# 6: StandUp          # 无参数
# 7: RecoveryStand    # 无参数
# 8: Once             # 单次运行旋转、位移[r,p,y,x,y,z]，在下游进行多次控制
uint8[9] control_mode # 可传入多个模式，在下游进行多次控制
# 控制量[仅与控制模式相关的控制量取值有效]
PathPoint[30] path_point
geometry_msgs/Twist point # x,y,yaw r,p,y 其中z可赋值为0
uint8 speed_level
float64 body_height