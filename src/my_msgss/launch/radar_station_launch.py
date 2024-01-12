from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_msgss'),
        'config',
        'radar.yaml'
    )
    config_1 = os.path.join(
        get_package_share_directory('my_msgss'),
        'config',
        'ros2_displayer.yaml'
    )
    get_img = Node(
        package='Get_Camera_Img',
        executable='Get_Img',
        name='Get_Img',
        output='screen'
    )
    img_handle = Node(
        package='Img_Handle',
        executable='Img_Sub',
        name='Img_Handle',
        output='screen'
    )
    get_depth =Node(
        package='get_depth',
        executable='get_depth_node',
        name='Get_Depth',
        output='screen'
    )
    game_map = Node(
        package='Game_Map',
        executable='Game_Map',
        name='game_map',
        output='screen',
        parameters=[config, config_1]
    )
    return LaunchDescription([game_map, get_img, img_handle, get_depth])


