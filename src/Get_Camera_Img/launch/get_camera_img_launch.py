from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    get_camera_img = Node(
        package='Get_Camera_Img',
        executable='Get_Img',
        name='Get_Img',
        output='screen'
    )
    return LaunchDescription([get_camera_img])