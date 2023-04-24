#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include "end_row_navigation/client.hpp"
#include <end_row_navigation/ParallelLineNavigationAction.h>
#include <algorithm>




	void  ParallelLineNavigationClient::spinThread()
	{
		ros::spin();
	}


	void ParallelLineNavigationClient::doneCB(const actionlib::SimpleClientGoalState& state, 
				const end_row_navigation::ParallelLineNavigationResult::ConstPtr& result)	
	{
		ROS_INFO("CLIENT ParallelLineNavigationClient: Action finished: %s. ", state.toString().c_str()); 
	}

	void  ParallelLineNavigationClient::activeCB()
	{
		ROS_INFO("ParallelLineNavigationClient: Goal just went active.");
	}

	void  ParallelLineNavigationClient::feedbackCB(const end_row_navigation::ParallelLineNavigationFeedback::ConstPtr& feedback)
	{
		ROS_INFO("ParallelLineNavigationClient: Got feedback. How is it possible?");
	}

	// Method to create a goal and send it to the server
	int  ParallelLineNavigationClient::start_client(ros::NodeHandle n, double max_speed, double min_travel_length, std::string server_name, int corridors_to_skip){
		
		//connect with server
		name=server_name;
		actionlib::SimpleActionClient<end_row_navigation::ParallelLineNavigationAction> ac(name);		
  		ac.waitForServer();

		//send a goal to the server
  		goal.min_travel_length=min_travel_length;
		goal.speed=max_speed;
		goal.corridors_to_skip=corridors_to_skip;

		ac.sendGoal(goal,
			boost::bind(&ParallelLineNavigationClient::doneCB, this,_1, _2),
			boost::bind(&ParallelLineNavigationClient::activeCB, this),
			boost::bind(&ParallelLineNavigationClient::feedbackCB, this, _1));

		//wait for server's results
		bool res=ac.waitForResult(ros::Duration(0.0));

		return res;

	}




