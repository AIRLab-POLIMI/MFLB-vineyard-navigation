#ifndef TURN_CLIENT
#define TURN_CLIENT

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turn/TurnAction.h>
#include <boost/thread.hpp>



class TurnClient{
	public:
			
		int start_client(double max_speed, double min_travel_length, int exit_enter, std::string server_name);

		
		void doneCB(const actionlib::SimpleClientGoalState& state, const turn::TurnResult::ConstPtr& result);

		void activeCB();

		void feedbackCB(const turn::TurnFeedback::ConstPtr& feedback);

		void spinThread();



	private:
		std::string name;
		ros::NodeHandle n;
		turn::TurnGoal goal;

};
#endif 
