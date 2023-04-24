#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turn/TurnAction.h>
#include <boost/thread.hpp>
#include "turn/client.hpp"
#include <algorithm>




	void  TurnClient::spinThread()
	{
		ros::spin();
	}


	void TurnClient::doneCB(const actionlib::SimpleClientGoalState& state, 
				const turn::TurnResult::ConstPtr& result)	
	{
		ROS_INFO("TurnClient: Action finished: %s. ", state.toString().c_str()); 
	}

	void  TurnClient::activeCB()
	{
		ROS_INFO("TurnClient: Goal just went active.");
	}

	void  TurnClient::feedbackCB(const turn::TurnFeedback::ConstPtr& feedback)
	{
		ROS_INFO("TurnClient: Got feedback. How is it possible?");
	}

	int  TurnClient::start_client(double default_turning_angle, double speed, int exit_enter, std::string server_name){
	
		//Connection with server
		name=server_name;
		actionlib::SimpleActionClient<turn::TurnAction> ac(name);
  		ac.waitForServer();

		//send a goal to the ActionServer
  		goal.default_turning_angle=default_turning_angle;
		goal.speed=speed;

		ac.sendGoal(goal,
			boost::bind(&TurnClient::doneCB, this,_1, _2),
			boost::bind(&TurnClient::activeCB, this),
			boost::bind(&TurnClient::feedbackCB, this, _1));


		//Wait for responce
		bool res=ac.waitForResult(ros::Duration(0.0));

		return res;

	}




