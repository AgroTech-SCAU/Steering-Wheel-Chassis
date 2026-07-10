import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='robot_cartographer_mapping').find('robot_cartographer_mapping')
    
    ld = LaunchDescription()

    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='robot_2d.lua')

    # ── 激光雷达驱动 ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'),
                         'launch', 'lsn10p_launch.py')
        )
    ))

    # ── MCU 通信桥接 ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mcu_comm_bridge'),
                         'launch', 'mcu_comm_bridge.launch.py')
        )
    ))

    # ── 机器人描述 (robot_state_publisher + joint_state_publisher) ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'),
                         'launch', 'robot_description.launch.py')
        )
    ))

    #=====================声明两个节点，cartographer/occupancy_grid_node=================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    #===============================================定义启动文件========================================================
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)


    return ld
