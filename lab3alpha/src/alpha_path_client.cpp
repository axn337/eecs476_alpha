//path_client:
// a code to send the desired poses to bath serves

#include <ros/ros.h>
#include <lab3alpha/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "alpha_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<lab3alpha::PathSrv>("alpha_path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    lab3alpha::PathSrv path_srv;
    
    //First pose

    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    pose.position.x = 1.0; //  desired x-coord is 3
    pose.position.y = 0.0;
    pose.position.z = 0.0; 
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    // initial Pose


    quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y=1.0; //  desired y-coord is 3
    //pose_stamped.pose.position.x=3.0; // same x
    path_srv.request.nav_path.poses.push_back(pose_stamped);



    quat = convertPlanarPhi2Quaternion(3.14); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;
    //pose_stamped.pose.position.y=2.0; // same y
    pose_stamped.pose.position.x=0.0; //  desired x-coord is 0.5
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // fourth Pose

    quat = convertPlanarPhi2Quaternion(-1.57); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y= 0.0; // desired y-coord is 8.0
    //pose_stamped.pose.position.x=6.5; // same x
    path_srv.request.nav_path.poses.push_back(pose_stamped);

   

    client.call(path_srv);

    return 0;
}
