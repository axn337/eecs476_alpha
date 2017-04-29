#include <ros/ros.h>
#include <final_lab/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <coordinator/OpenLoopNavSvc.h>
#include <final_lab/path.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
using namespace std;

geometry_msgs::PoseStamped g_perceived_object_pose;
ros::Publisher *g_pose_publisher;
geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
coordinator::ManipTaskResult g_result;

int g_found_object_code;
bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code=0;
int g_object_finder_return_code=0;
int g_fdbk_count = 0;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<final_lab::path>("append_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    ROS_INFO("Creating path trajectory");
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    final_lab::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 3.7; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = 2.1;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    client.call(path_srv);

    //wait for robot to reach stool
    ROS_INFO("Waiting for Jynx to reach the stool");
    while(path_srv.response.status!=true){
	ros::Duration(0.1).sleep();
    }

     ROS_INFO("path executed");

    ////////////////////////////////////////////////
    coordinator::OpenLoopNavSvc openLoopNavSvcMsg;

    ros::ServiceClient nav_move_client = n.serviceClient<coordinator::OpenLoopNavSvc>("open_loop_nav_service");

    
    ROS_INFO("backing up");
    openLoopNavSvcMsg.request.move_distance= -1.0; // back up 1m
    nav_move_client.call(openLoopNavSvcMsg);
    ros::Duration(1.0).sleep(); //wait to settle down
    



/////////////////////////////////////////////////////////

    ROS_INFO("Time to go back");

    final_lab::path path_srv_return;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    pose.position.x = 3.7; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv_return.request.path.poses.push_back(pose_stamped);
 
    pose.position.x = 0.0;
    pose_stamped.pose = pose;
    path_srv_return.request.path.poses.push_back(pose_stamped);

    client.call(path_srv_return);
    return 0;
}
