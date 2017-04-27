#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
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
#include <mobot_pub_des_state/path.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
using namespace std;

geometry_msgs::PoseStamped g_perceived_object_pose;
ros::Publisher *g_pose_publisher;

int g_found_object_code;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    ROS_INFO("Creating path trajectory");
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 4.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = 2.5;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    client.call(path_srv);

    //wait for robot to reach stool
    ROS_INFO("Waiting for Jynx to reach the stool");

    ////////////////////////////////////////////////

    //Start finding block
    coordinator::ManipTaskGoal goal;
    coordinator::OpenLoopNavSvc openLoopNavSvcMsg;
    tf::TransformListener tfListener;
    XformUtils xform_utils; //instantiate an object of XformUtils

    actionlib::SimpleActionClient<coordinator::ManipTaskAction> action_client("manip_task_action_service", true);
    ros::ServiceClient nav_move_client = nh.serviceClient<coordinator::OpenLoopNavSvc>("open_loop_nav_service");

    // attempt to connect to the server:
    ROS_INFO("waiting for the manipulation action server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = action_client.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    ROS_INFO("sending a goal: move arms to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move quitting");
        return 0;
    }
    //send vision request to find table top:
    ROS_INFO("sending a goal: seeking table top");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::FIND_TABLE_SURFACE;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    
    //send vision goal to find block:
    ROS_INFO("sending a goal: find block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GET_PICKUP_POSE;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
    goal.perception_source = coordinator::ManipTaskGoal::PCL_VISION;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to find block quitting");
        return 0;
    }
    g_object_pose = g_result.object_pose;
    ROS_INFO_STREAM("object origin: (x,y,z) = ("<<g_object_pose.pose.position.x<<", "<<g_object_pose.pose.position.y<<", "
              <<g_object_pose.pose.position.z<<")"<<endl);
    ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = ("<<g_object_pose.pose.orientation.x<<","
              <<g_object_pose.pose.orientation.y<<","
              <<g_object_pose.pose.orientation.z<<","
              <<g_object_pose.pose.orientation.w<<")"<<endl);    
    
    //send command to acquire block:
    ROS_INFO("sending a goal: grab block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GRAB_OBJECT;
    goal.pickup_frame = g_result.object_pose;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
    //goal.perception_source= coordinator::ManipTaskGoal::BLIND_MANIP;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to grab block; quitting");
        return 0;
    }
    
    ROS_INFO("sending a goal: move arms to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move to pre-pose; quitting");
        return 0;
    }
    
    ROS_INFO("backing up");
    openLoopNavSvcMsg.request.move_distance= -1.0; // back up 1m
    nav_move_client.call(openLoopNavSvcMsg);
    ros::Duration(1.0).sleep(); //wait to settle down
    tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);    
    double yaw = xform_utils.convertPlanarQuat2Phi(current_pose.pose.orientation);
    ROS_INFO("yaw = %f",yaw);



/////////////////////////////////////////////////////////

    ROS_INFO("Time to go back");

    mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 4.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = 2.5;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    client.call(path_srv);
    return 0;
}
