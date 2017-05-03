//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service

#include <ros/ros.h>
#include <final_lab/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

using namespace std;
bool path_done=false;


void doneCB(const std_msgs::Bool& done){
	  
    //ROS_INFO("path isn't done yet"); 

    //ros::Duration(0.1).sleep();

    //while(!path_done){
    //ros::Duration(0.1).sleep();
    
   // if(done.data){
        path_done=done.data;
    //}
   // ROS_INFO("topic path_done is publishing %d", done.data); 

    //}

    //ROS_INFO("path executed");
}
	  
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
    
    path_done=false;

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
	
	/*/
    pose.position.y = 2.1;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    //return path


    pose.position.y = 0.0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.x = 0.0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    /*/
    
    client.call(path_srv);

    ros::Subscriber path_done_subscriber= n.subscribe("path_done",1,doneCB); 

    while(path_done!=true){
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ROS_INFO("path executed");
 

	

    return 0;
}
