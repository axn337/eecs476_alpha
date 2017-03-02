//Kevin Bradner's action client for mobile robotics ps4
//much of this based on the learning_ros example_action_client

#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include<kmb172_ps4/p4msgAction.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>

bool alarm_active = false;//a global var to hold alarm information

//just here because a line near the bottom points to it
void doneCb(const actionlib::SimpleClientGoalState& state, const kmb172_ps4::p4msgResultConstPtr& result) {
	ROS_INFO("DONE");
}

//TODO: fix this function, which caused issues
void alarm_Callback(const std_msgs::Bool& message_holder){
	if(!alarm_active){
		alarm_active = false;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "path_action_client");//make node
	ros::NodeHandle n;

	bool cancellation = false;//whether a goal cancellation has been ordered
	kmb172_ps4::p4msgGoal altGoal;//in case the first goal is cancelled

	//make a subscriber that keeps track of the alarm state
	//alarm status gets stored in alarm_active
	//TODO: fix the following line, it was causing issues
	ros::Subscriber alarm_monitor = n.subscribe("lidar_alarm",1,alarm_Callback);

	kmb172_ps4::p4msgGoal goal;//create goal object to pack message into
	actionlib::SimpleActionClient<kmb172_ps4::p4msgAction> action_client("p4_action", true);
	
	double pi = 3.142;//for easy phi values

        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(15.0)); // wait

	if (!server_exists) {
			ROS_WARN("could not connect to server; halting");
        		return 0; // bail out; optionally, could print a warning message and retry
    	}

	ROS_INFO("connected to action server"); // if here, then we connected to the server;

	//fill some arrays with magic numbers corresponding to the desired motions
	//initial pose command
	double next_x_move = 0.0;
	double next_y_move = 0.0;
	double next_orientation = 0.0;
	
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);

	//face north and add to goal
	next_orientation = (0.0);
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);


	//east 1
	next_orientation = (0.0);
	next_x_move = 0.2;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (0.0);
	next_x_move = 0.2;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (0.0);
	next_x_move = 0.2;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (0.0);
	next_x_move = 0.2;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (0.0);
	next_x_move = 0.2;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);


	//north 1
	next_orientation = (pi / 2.0);
	next_x_move = 0.0;
	next_y_move = 0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi / 2.0);
	next_x_move = 0.0;
	next_y_move = 0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi / 2.0);
	next_x_move = 0.0;
	next_y_move = 0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi / 2.0);
	next_x_move = 0.0;
	next_y_move = 0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi / 2.0);
	next_x_move = 0.0;
	next_y_move = 0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);


	//west 1
	next_orientation = (pi);
	next_x_move = -0.20;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi);
	next_x_move = -0.20;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi);
	next_x_move = -0.20;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi);
	next_x_move = -0.20;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (pi);
	next_x_move = -0.20;
	next_y_move = 0.0;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);


	//south 1
	next_orientation = (3.0 * pi / 2.0);
	next_x_move = 0.0;
	next_y_move = -0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (3.0 * pi / 2.0);
	next_x_move = 0.0;
	next_y_move = -0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (3.0 * pi / 2.0);
	next_x_move = 0.0;
	next_y_move = -0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (3.0 * pi / 2.0);
	next_x_move = 0.0;
	next_y_move = -0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);
	next_orientation = (3.0 * pi / 2.0);
	next_x_move = 0.0;
	next_y_move = -0.20;
	goal.x_coords.push_back(next_x_move);
	goal.y_coords.push_back(next_y_move);
	goal.phi_vals.push_back(next_orientation);


	

	//send the goal for execution
	action_client.sendGoal(goal, &doneCb);
	ROS_INFO("Goal message constructed and sent for execution");
	
	//monitor the lidar alarm to see if a cancellation is needed
	while(true){
		bool motion_completed = action_client.waitForResult(ros::Duration(0.1));

		if(motion_completed){
			ROS_INFO("motion completed successfully");
			break;
		}
		
		ROS_INFO("Checking for alarm");
		if(alarm_active){//goal cancellation needed
			action_client.cancelGoal();//cancel the goal
			cancellation = true;
			ROS_INFO("sending goal cancellation to action server");
			break;
		}

		ros::spinOnce();
	}
	
	if(cancellation){//spin in place if the goal was cancelled

		ROS_INFO("spinning in place due to goal cancellation");		
			
		next_orientation = (0.0);
		next_x_move = 0.0;
		next_y_move = 0.0;
		
		altGoal.x_coords.push_back(next_x_move);
		altGoal.y_coords.push_back(next_y_move);
		altGoal.phi_vals.push_back(next_orientation);

		next_orientation = (pi / 2.0);
		altGoal.x_coords.push_back(next_x_move);
		altGoal.y_coords.push_back(next_y_move);
		altGoal.phi_vals.push_back(next_orientation);

		next_orientation = (pi);
		altGoal.x_coords.push_back(next_x_move);
		altGoal.y_coords.push_back(next_y_move);
		altGoal.phi_vals.push_back(next_orientation);

		next_orientation = (pi / 2.0);
		altGoal.x_coords.push_back(next_x_move);
		altGoal.y_coords.push_back(next_y_move);
		altGoal.phi_vals.push_back(next_orientation);

		next_orientation = (0.0);
		altGoal.x_coords.push_back(next_x_move);
		altGoal.y_coords.push_back(next_y_move);
		altGoal.phi_vals.push_back(next_orientation);

		action_client.sendGoal(altGoal, &doneCb);
		
	}
	
	return 0;
}
