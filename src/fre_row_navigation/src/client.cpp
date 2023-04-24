#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fre_row_navigation/CrawlRowAction.h>
#include <boost/thread.hpp>
#include "fre_row_navigation/client.hpp"
#include <algorithm>




	void  RowCrawlClient::spinThread()
	{
		ros::spin();
	}


	void RowCrawlClient::doneCB(const actionlib::SimpleClientGoalState& state, 
				const fre_row_navigation::CrawlRowResult::ConstPtr& result)	
	{
		ROS_INFO("CrawlRowClient: Action finished: %s.", state.toString().c_str()); 
	}

	void  RowCrawlClient::activeCB()
	{
		ROS_INFO("CrawlRowClient: Goal just went active.");
	}

	void  RowCrawlClient::feedbackCB(const fre_row_navigation::CrawlRowFeedback::ConstPtr& feedback)
	{
		ROS_INFO("CrawlRowClient: Got feedback. How is it possible?");
	}

	void  RowCrawlClient::start_client(ros::NodeHandle n, double max_speed, double min_row_length, std::string server_name){
	
		//contact server
		name=server_name;
		actionlib::SimpleActionClient<fre_row_navigation::CrawlRowAction> ac(name);		
  		ac.waitForServer();

		//send a goal to the action
  		goal.min_row_length=min_row_length;
		goal.speed=max_speed;
		goal.todo_rows;

		ac.sendGoal(goal,
			boost::bind(&RowCrawlClient::doneCB, this,_1, _2),
			boost::bind(&RowCrawlClient::activeCB, this),
			boost::bind(&RowCrawlClient::feedbackCB, this, _1));

		
		//wait until server sends a response
		bool res=ac.waitForResult(ros::Duration(0.0));
		//goal is finished

	}

