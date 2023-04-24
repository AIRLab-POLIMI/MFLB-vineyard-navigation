#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pcl_assembler/client.hpp>
#include <pcl_assembler/PclAssemblerAction.h>
#include <boost/thread.hpp>
#include "pcl_assembler/client.hpp"
#include <algorithm>




	void  PclAssemblerClient::spinThread()
	{
		ros::spin();
	}


	void PclAssemblerClient::doneCB(const actionlib::SimpleClientGoalState& state, 
				const pcl_assembler::PclAssemblerResult::ConstPtr& result)	
	{
		ROS_INFO("PclAssemblerClient: Action finished: %s. ", state.toString().c_str()); 
		//clusters_result=result->clusters;
	}

	void  PclAssemblerClient::activeCB()
	{
		//ROS_INFO("PclAssemblerClient: Goal just went active.");
	}

	void  PclAssemblerClient::feedbackCB(const pcl_assembler::PclAssemblerFeedback::ConstPtr& feedback)
	{
		//("PclAssemblerClient: Got feedback. How is it possible?");
	}

	// Method to send a request to the ActionServer
	void PclAssemblerClient::get_clusters(std::string server_name, int start){
		//Connect to server 
		name=server_name;
		actionlib::SimpleActionClient<pcl_assembler::PclAssemblerAction> ac(name);		
  		ac.waitForServer();

		//send a goal to the action
  		goal.start=start;

		ac.sendGoal(goal,
			boost::bind(&PclAssemblerClient::doneCB, this,_1, _2),
			boost::bind(&PclAssemblerClient::activeCB, this),
			boost::bind(&PclAssemblerClient::feedbackCB, this, _1));

		// no need to wait for a result


	}


