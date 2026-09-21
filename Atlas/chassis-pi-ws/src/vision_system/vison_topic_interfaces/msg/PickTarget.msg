# Correlates this request with /pick_result
uint32 request_id
# Cancel continuation for this request; does not stop already accepted arm motion
bool cancel

# 选择用哪个角 + 第几层高度
int32 corner_index
int32 layer

# true 时使用 target_z_m 替代 handeye_bridge 全局 planeX_z_m
bool use_target_z
float64 target_z_m

# Legacy fields retained for source migration; all nodes must rebuild this message
# handeye uses detection-synchronized tool z
bool use_orientation
float64 pitch_rad
float64 yaw_rad

# true 时 target_z_m 表示实际吸取接触高度，approach_m 表示先到达其上方的距离
bool use_approach
float64 approach_m
