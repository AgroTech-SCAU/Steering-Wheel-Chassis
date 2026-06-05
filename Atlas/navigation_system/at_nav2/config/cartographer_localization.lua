include "map_builder.lua"                                                                             -- 引入地图构建器基础配置
include "trajectory_builder.lua"                                                                      -- 引入轨迹构建器基础配置

options = {
  map_builder = MAP_BUILDER,                                                                          -- 地图构建器：使用默认MAP_BUILDER
  trajectory_builder = TRAJECTORY_BUILDER,                                                            -- 轨迹构建器：使用默认TRAJECTORY_BUILDER
  map_frame = "map",                                                                                  -- 地图坐标系：全局固定坐标系
  tracking_frame = "base_link",                                                                       -- 跟踪坐标系：机器人本体坐标系，用于位姿估计
  published_frame = "odom",                                                                           -- 发布坐标系：Cartographer发布的位姿所属坐标系
  odom_frame = "odom",                                                                                -- 里程计坐标系：底盘里程计所在的坐标系
  provide_odom_frame = false,                                                                         -- 是否提供odom坐标系：false表示odom由外部（底盘odom_driver）提供
  publish_frame_projected_to_2d = false,                                                              -- 是否将3D位姿投影到2D发布：3D位姿直接发布，不做投影
  use_odometry = true,                                                                                -- 是否使用里程计数据：true，启用轮式里程计作为运动先验
  use_nav_sat = false,                                                                                -- 是否使用GPS/卫星导航：false，不使用
  use_landmarks = false,                                                                              -- 是否使用地标：false，不使用
  num_laser_scans = 1,                                                                                -- 2D激光雷达数量：1个（LSN10P单线激光）
  num_multi_echo_laser_scans = 0,                                                                     -- 多回波激光雷达数量：0，不使用
  num_subdivisions_per_laser_scan = 1,                                                                -- 每帧激光扫描的细分数量：1，不做细分
  num_point_clouds = 0,                                                                               -- 3D点云数量：0，不使用3D点云
  lookup_transform_timeout_sec = 0.2,                                                                 -- TF变换查找超时时间(s)：0.2秒
  submap_publish_period_sec = 0.3,                                                                    -- 子图发布周期(s)：每0.3秒发布一次子图
  pose_publish_period_sec = 5e-3,                                                                     -- 位姿发布周期(s)：5ms（200Hz）高频发布定位结果
  trajectory_publish_period_sec = 30e-3,                                                              -- 轨迹发布周期(s)：30ms（约33Hz）
  rangefinder_sampling_ratio = 1.,                                                                    -- 测距仪数据采样比例：1.0 表示全部采样，不降采样
  odometry_sampling_ratio = 1.,                                                                       -- 里程计数据采样比例：1.0 表示全部采样
  fixed_frame_pose_sampling_ratio = 1.,                                                               -- 固定坐标系位姿采样比例：1.0 表示全部采样
  imu_sampling_ratio = 1.,                                                                            -- IMU数据采样比例：1.0 表示全部采样（注意当前未启用IMU） 位于第36行
  landmarks_sampling_ratio = 1.,                                                                      -- 地标数据采样比例：1.0 表示全部采样（注意当前未使用地标）
} 

MAP_BUILDER.use_trajectory_builder_2d = true                                                          -- 使用2D轨迹构建器：2D模式建图/定位

TRAJECTORY_BUILDER_2D.min_range = 0.4                                                                 -- 激光最小有效距离(m)：0.4m，过滤近距离噪点
TRAJECTORY_BUILDER_2D.max_range = 3.5                                                                 -- 激光最大有效距离(m)：3.5m，匹配LSN10P激光雷达量程
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0                                                   -- 缺失数据射线长度(m)：5m，激光无回波时假想的射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = false                                                            -- 是否使用IMU数据：false，本车不使用IMU
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true                                     -- 是否启用在线相关扫描匹配：true，实时CSM提高定位精度
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1                   -- 实时CSM线性搜索窗口(m)：±0.1m
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)        -- 实时CSM角度搜索窗口(rad)：±20°
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1         -- 实时CSM平移代价权重：0.1，惩罚偏离初始位姿的平移量
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1            -- 实时CSM旋转代价权重：0.1，惩罚偏离初始位姿的旋转量

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3                                            -- 静止时最多0.3秒才取一帧
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1                                         -- 或移动0.1m才取一帧
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)                                  -- 或旋转1度才取一帧

POSE_GRAPH.optimization_problem.huber_scale = 1e2                                                     -- 位姿图优化Huber损失尺度：100，降低外点对优化的影响
POSE_GRAPH.optimize_every_n_nodes = 35                                                                -- 位姿图优化频率：每35个节点执行一次全局优化
POSE_GRAPH.constraint_builder.min_score = 0.55                                                        -- 约束构建最小匹配分数：0.65，低于此分数的回环/约束被丢弃

TRAJECTORY_BUILDER.pure_localization_trimmer = {                                                      -- [HW_CONFIG] Pure localization mode: does not build new map, only localizes
  max_submaps_to_keep = 5,                                                                            -- 纯定位模式：不建立新地图，仅在已有地图上进行定位
}

-- Load pre-built .pbstream map (path specified in launch file)
-- 加载预建好的 .pbstream 地图（路径由 launch 文件指定）
return options                                                                                        -- 返回配置表给Cartographer节点

