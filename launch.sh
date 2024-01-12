colcon build --symlink-install
cmds=( "ros2 launch Get_Camera_Img get_camera_img_launch.py"
        "ros2 launch Img_Handle img_handle_launch.py"
        "ros2 launch get_depth get_depth_launch.py"
        "ros2 launch Game_Map game_map_launch.py")

cmd=("ros2 launch livox_ros2_driver livox_lidar_launch.py")

for cmd in "${cmd[@]}";
do
    gnome-terminal -- bash -c "cd /home/livox_ros2_driver/livox_ros2_driver;source install/setup.bash;$cmd;exec bash;"
    sleep 0.2
done

for cmd in "${cmds[@]}"; 
do
    echo Current CMD : "$cmd"
  	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
    sleep 0.2
done