from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
    game_map = Node(
        package='Game_Map',
        executable='Game_Map',
        name='Game_Map',
        output='screen'
    )
    return LaunchDescription([get_img, img_handle, game_map])