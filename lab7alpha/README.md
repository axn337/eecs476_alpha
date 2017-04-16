# my_lidar_alarm

A submession for assignment 6 in EECS476. Refer to the write up.

# Running the program

roslaunch gazebo_ros empty_world.launch
roslaunch exmpl_models add_glennan_2nd_flr.launch
roslaunch mobot_urdf mobot_w_lidar.launch


-----------------------------------
export ROS_IP=129.22.143.20
export ROS_MASTER_URI=http://129.22.148.227:11311

on jinx:
export ROS_MASTER_URI=http://129.22.148.227:11311
export ROS_IP=129.22.148.227

--------------------------------------------
cd ~/ros_ws/src/learning_ros/maps/gl2_map/, then:
rosrun map_server map_server gl2_map.yaml
rosrun amcl amcl
rosrun mobot_drifty_odom mobot_drifty_odom
---rosrun odom_tf odom_tf_demo drifty_odom:=odom
rosrun odom_tf odom_tf_demo

rosrun lab7alpha alpha_lidar_alarm

open loop:
rosrun lab7alpha alpha_open_loop_controller

lin_steering:
rosrun lab7alpha alpha_lin_steering

Running Publisher ans client:
rosrun lab7alpha alpha_pub_des_state_path_client
rosrun lab7alpha alpha_pub_des_state




rosrun example_rviz_marker triad_display


