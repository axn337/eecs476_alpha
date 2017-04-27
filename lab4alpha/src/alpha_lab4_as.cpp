//Kevin Bradner's action server for mobile robotics ps4
//takes and executes a set of open loop control commands

//based heavily on the example_action_server from learning_ros

#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<kmb172_ps4/p4msgAction.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>//pow

// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
ros::Publisher g_twist_commander;

bool alarm_active = false;

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

void alarm_Callback(const std_msgs::Bool& message_holder){
	alarm_active = message_holder.data;
}

class ps4ActionServer{
private:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<kmb172_ps4::p4msgAction> as_;
	kmb172_ps4::p4msgGoal goal_;
	kmb172_ps4::p4msgResult result_;
	kmb172_ps4::p4msgFeedback feedback_;//not sure how to actually use this or if I need to

public:
	ps4ActionServer();
	
	~ps4ActionServer(void) {
	}

	void executeCB(const actionlib::SimpleActionServer<kmb172_ps4::p4msgAction>::GoalConstPtr& goal);
};//what's up with the syntax on this line?

ps4ActionServer::ps4ActionServer() ://TODO: correct for this package?
	as_(nh_, "p4_action", boost::bind(&ps4ActionServer::executeCB, this, _1),false)
{
	ROS_INFO("in constructor of action server");
	//TODO: do I need any initializations here?
	as_.start();
}

void ps4ActionServer::executeCB(const actionlib::SimpleActionServer<kmb172_ps4::p4msgAction>::GoalConstPtr& goal){
	double theta_current = 0.0;//to track current orientation
	
	int npts = goal->x_coords.size();//get number of poses to be visited
	double dist = 0.0;
	double delta_theta = 0.0;

	for(int i=0;i<npts;i++){
		dist = sqrt(pow(goal->x_coords[i],2.0)+pow(goal->y_coords[i],2.0));//calculate the next move distance
		delta_theta = (goal->phi_vals[i]) - theta_current;//calculate change in angle
		do_spin(1.5 * delta_theta);//assume correct angle
		theta_current = goal->phi_vals[i];//update current joint angle
		do_move(dist);//command the move
		if (as_.isPreemptRequested()){	
			ROS_WARN("goal cancelled!");
			result_.completed = false;
			as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
			return; // done with callback
		}
		ros::spinOnce();//give chance to check for lidar alarm
		while(alarm_active){
			ros::spinOnce();
		}
	}

	result_.completed = true;
	as_.setSucceeded(result_);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "p4_action_server");
	ros::NodeHandle nh_;
	
	g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //global publisher object
	
	ros::Subscriber alarm_monitor = nh_.subscribe("alpha_lidar_alarm",1,alarm_Callback);
	
	ps4ActionServer as_object;

	ROS_INFO("going into spin");
	ros::spin();
	return 0;
}

//below here, all from the example service from ps3
using namespace std;
//some tunable constants, global
const double g_move_speed = 0.3; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.5; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}
