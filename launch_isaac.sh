#!/bin/bash
source ~/env_isaaclab/bin/activate
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
INTERNAL_ROS2=$(find ~/env_isaaclab -path "*/isaacsim.ros2.bridge/humble" -type d | head -1)
export LD_LIBRARY_PATH=$INTERNAL_ROS2/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link &

unset PYTHONPATH
isaacsim --exec "/home/ghost/Isaac_sim_python_codes/load_scene.py"
#python3 /home/ghost/Isaac_sim_python_codes/sim_rgb_depth_point_tf.py
#isaacsim --exec "/home/ghost/Isaac_sim_python_codes/setup_camera.py"

#isaacsim --exec "open_stage /home/ghost/Downloads/fireloop_assignment/isaac-usd/isaac_sim_scene/scene.usda" 

