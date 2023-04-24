#ifndef PARALLEL_LINE_NAVIGATION_CLIENT
#define PARALLEL_LINE_NAVIGATION_CLIENT

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <end_row_navigation/ParallelLineNavigationAction.h>
#include <boost/thread.hpp>



class ParallelLineNavigationClient{
	public:
			
		int start_client(ros::NodeHandle n, double max_speed, double min_travel_length, std::string server_name, int corridors_to_skip);

		
		void doneCB(const actionlib::SimpleClientGoalState& state, const end_row_navigation::ParallelLineNavigationResult::ConstPtr& result);

		void activeCB();

		void feedbackCB(const end_row_navigation::ParallelLineNavigationFeedback::ConstPtr& feedback);

		void spinThread();



	private:
		std::string name;
		ros::NodeHandle n;
		end_row_navigation::ParallelLineNavigationGoal goal;

};
#endif 
