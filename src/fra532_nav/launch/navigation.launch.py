import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # กำหนดตำแหน่งของแพ็คเกจที่เกี่ยวข้อง
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # เปลี่ยนชื่อแพ็คเกจด้านล่างให้ตรงกับแพ็คเกจของคุณที่เก็บ map และ parameter
    my_nav_pkg_dir = get_package_share_directory('fra532_nav')
    
    # กำหนดตำแหน่งไฟล์ configuration ต่างๆ
    rviz_config_file = os.path.join(my_nav_pkg_dir, 'rviz', 'navigation.rviz')
    map_yaml_file = os.path.join(my_nav_pkg_dir, 'maps', 'map.yaml')
    params_file = os.path.join(my_nav_pkg_dir, 'params', 'basic_params.yaml')
    
    # สร้าง LaunchConfigurations สำหรับการกำหนด argument
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ประกาศ Launch Arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Set to "True" to enable SLAM; "False" to use a pre-built map'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    # รวม launch file ของ nav2_bringup ซึ่งจะเริ่ม node หลักของ navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )
    
    # รวม RViz launch สำหรับการดูสถานะของ navigation (เลือกใช้งานได้)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file
        }.items()
    )
    
    # UNCOMMENT HERE FOR KEEPOUT DEMO
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[params_file])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[params_file])
    
    # สร้าง launch description และเพิ่ม action ทั้งหมดลงไป
    ld = LaunchDescription()
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    # UNCOMMENT HERE FOR KEEPOUT DEMO
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)
    
    return ld


# # Copyright (c) 2021 Samsung Research America
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch_ros.actions import Node


# def generate_launch_description():
#     # Get the launch directory
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     bringup_dir = get_package_share_directory('fra532_nav')
#     rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

#     nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

#     # Create the launch configuration variables
#     slam = LaunchConfiguration('slam')
#     map_yaml_file = LaunchConfiguration('map')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     params_file = LaunchConfiguration('params_file')

#     # Declare the launch arguments

#     declare_slam_cmd = DeclareLaunchArgument(
#         'slam',
#         default_value='False',
#         description='Whether run a SLAM')
    
#     declare_map_yaml_cmd = DeclareLaunchArgument(
#         'map',
#         default_value=os.path.join(
#             bringup_dir, 'maps', 'map.yaml'),
#         description='Full path to map file to load')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true')

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(bringup_dir, 'params', 'basic_params.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes')

#     bringup_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_launch_dir, 'bringup_launch.py')),
#         launch_arguments={'slam': slam,
#                           'map': map_yaml_file,
#                           'use_sim_time': use_sim_time,
#                           'params_file': params_file,
#                           'autostart': 'True'}.items())

#     rviz_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_launch_dir, 'rviz_launch.py')),
#         launch_arguments={'namespace': '',
#                           'use_sim_time': use_sim_time,
#                           'rviz_config': rviz_config_file}.items())
    
#     # UNCOMMENT HERE FOR KEEPOUT DEMO
#     # start_lifecycle_manager_cmd = Node(
#     #         package='nav2_lifecycle_manager',
#     #         executable='lifecycle_manager',
#     #         name='lifecycle_manager_costmap_filters',
#     #         output='screen',
#     #         emulate_tty=True,
#     #         parameters=[{'use_sim_time': use_sim_time},
#     #                     {'autostart': True},
#     #                     {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

#     # start_map_server_cmd = Node(
#     #         package='nav2_map_server',
#     #         executable='map_server',
#     #         name='filter_mask_server',
#     #         output='screen',
#     #         emulate_tty=True,
#     #         parameters=[params_file])

#     # start_costmap_filter_info_server_cmd = Node(
#     #         package='nav2_map_server',
#     #         executable='costmap_filter_info_server',
#     #         name='costmap_filter_info_server',
#     #         output='screen',
#     #         emulate_tty=True,
#     #         parameters=[params_file])

#     # Create the launch description and populate
#     ld = LaunchDescription()

#     # Declare the launch options
#     ld.add_action(declare_slam_cmd)
#     ld.add_action(declare_map_yaml_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(bringup_cmd)
#     ld.add_action(rviz_cmd)
#     # UNCOMMENT HERE FOR KEEPOUT DEMO
#     # ld.add_action(start_lifecycle_manager_cmd)
#     # ld.add_action(start_map_server_cmd)
#     # ld.add_action(start_costmap_filter_info_server_cmd)

#     return ld
