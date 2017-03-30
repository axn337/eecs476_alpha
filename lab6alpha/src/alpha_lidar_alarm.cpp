#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message
#include <std_srvs/Trigger.h>
#include <stdlib.h>     /* abs */
#include <cmath>//pow


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool triggered_ = false;
std_srvs::Trigger srv;

//new variables for my code below
float ping_dist_current_angle_ = 3.0;
float current_alarm_threshold_ = 0.0;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::ServiceClient lidar_trigger_;
ros::ServiceClient lidar_reset_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }

   //EDIT: modified the original code here so that it checks many angles
   //as the ping index is dead ahead, and the angle increment is 1/1000 turn
	//then check from ping_index_ - 150 to ping_index + 150 for the front section of robot

   for (int i = (ping_index_ - 80); i < (ping_index_ + 80); ++i){
	ping_dist_current_angle_ = laser_scan.ranges[i];//get range at angle of interest
	current_alarm_threshold_ = 1.0 - pow((abs(i - ping_index_) / 250),3);//varies from 0 to 1 by angle
  current_alarm_threshold_ *= 3.0;
  //current_alarm_threshold_ = MIN_SAFE_DISTANCE;
	//ROS_INFO("ping dist at current angle = %f",ping_dist_current_angle_);
        if (ping_dist_current_angle_<current_alarm_threshold_) {//check against angle specific threshold
            ROS_WARN("DANGER %f", laser_scan.ranges[181]);
            laser_alarm_=true;
            triggered_=true;
            lidar_trigger_.call(srv);
            break;//exit the loop: don't need to check for more alarms
        }
        else {
           laser_alarm_=false;
        }
   }



   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
   if (laser_alarm_==false && triggered_==true){
      lidar_reset_.call(srv);
      triggered_=false;
   }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    lidar_trigger_ = nh.serviceClient<std_srvs::Trigger>("estop_service");
    lidar_reset_ = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}