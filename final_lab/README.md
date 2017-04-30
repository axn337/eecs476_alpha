# final_lab

A submession for final lab assignment in EECS476. Refer to the write up.

## Example usage

## Running tests/demos

on Atlas: #atlas1 
export ROS_IP=129.22.143.20
 #atlas3 
export ROS_IP=129.22.143.219 
export ROS_MASTER_URI=http://129.22.148.227:11311

on jinx: export ROS_MASTER_URI=http://129.22.148.227:11311 export ROS_IP=129.22.148.227 roslaunch launchers start_jinx.launch

----------------------------------------

roslaunch gazebo_ros empty_world.launch
roslaunch exmpl_models add_glennan_2nd_flr.launch 
roslaunch mobot_urdf mobot_w_lidar.launch
---------------------------------
    
cd ~/ros_ws/src/learning_ros/maps/gl2_map/
then:
rosrun map_server map_server gl2_map.yaml rosrun amcl amcl rosrun mobot_drifty_odom mobot_drifty_odom 
rosrun odom_tf odom_tf_demo drifty_odom:=odom

lidar_alarm for publisher: 
rosrun final_lab alpha_lidar_alarm

open loop: 
rosrun final_lab alpha_open_loop_controller

lin_steering: 
rosrun final_lab alpha_lin_steering 


Running Publisher and client: 
rosrun final_lab alpha_pub_des_state_path_client 
rosrun final_lab alpha_pub_des_state

rosrun coordinator open_loop_nav_service

rosrun final_lab alpha_coordinator_no_arms

rosrun example_rviz_marker triad_display
