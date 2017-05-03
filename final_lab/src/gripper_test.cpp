#include <ros/ros.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_test");
    ros::NodeHandle n;
    ros::Publisher gripper = n.advertise<std_msgs::Bool>("/robot/close_gripper", 1);
    std_msgs::Bool grab;


    //grab
    
    grab.data = true;
    ros::Time start =  ros::Time::now();   
    while((ros::Time::now() - start) < ros::Duration(3)) {
    	gripper.publish(grab);
    	//ros::Duration(0.1);
    	//ros::spinOnce();
    }

    ROS_INFO("moving on");
    ros::Duration(1);

    grab.data = false;
    start =  ros::Time::now();   
    while((ros::Time::now() - start) < ros::Duration(3)) {
    	gripper.publish(grab);
    	//ros::Duration(0.1);
    	//ros::spinOnce();
    }

    return 0;
}
