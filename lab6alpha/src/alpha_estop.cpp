#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <std_msgs/Bool.h> // boolean message
#include <std_srvs/Trigger.h>

bool triggered_ = false;
std_srvs::Trigger srv;

ros::ServiceClient estop_trigger_;
ros::ServiceClient estop_reset_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void estopCallback(const std_msgs::Bool& estop) {
    
    if (estop.data) {//check against estop
      ROS_WARN("ESTOP!");
      triggered_=true;
      estop_trigger_.call(srv);
    }
    else if (triggered_){
      estop_reset_.call(srv);
      triggered_=false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "alpha_estop"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Subscriber estop_subscriber = nh.subscribe("/ESTOP", 1, estopCallback);
    estop_trigger_ = nh.serviceClient<std_srvs::Trigger>("estop_service");
    estop_reset_ = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}