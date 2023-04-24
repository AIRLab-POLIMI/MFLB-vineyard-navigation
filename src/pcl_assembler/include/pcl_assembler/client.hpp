#ifndef PCL_ASSEMBLER
#define PCL_ASSEMBLER

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pcl_assembler/PclAssemblerAction.h>
#include <boost/thread.hpp>



class PclAssemblerClient{
	public:
			
		void get_clusters(std::string server_name, int start);

		//void stop_searching_clusters();

		
		void doneCB(const actionlib::SimpleClientGoalState& state, const pcl_assembler::PclAssemblerResult::ConstPtr& result);

		void activeCB();

		void feedbackCB(const pcl_assembler::PclAssemblerFeedback::ConstPtr& feedback);

		void spinThread();



	private:
		std::string name;
		ros::NodeHandle n;
		pcl_assembler::PclAssemblerGoal goal;
};
#endif 
