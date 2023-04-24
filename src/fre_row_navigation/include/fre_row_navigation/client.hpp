#ifndef FRE_ROW_CRAWL_CLIENT
#define FRE_ROW_CRAWL_CLIENT

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fre_row_navigation/CrawlRowAction.h>
#include <boost/thread.hpp>



class RowCrawlClient{
	public:
			
		void start_client(ros::NodeHandle n, double max_speed, double min_row_length, std::string server_name);

		
		void doneCB(const actionlib::SimpleClientGoalState& state, const fre_row_navigation::CrawlRowResult::ConstPtr& result);

		void activeCB();

		void feedbackCB(const fre_row_navigation::CrawlRowFeedback::ConstPtr& feedback);

		void spinThread();


	private:
		std::string name;
		ros::NodeHandle n;
		fre_row_navigation::CrawlRowGoal goal;



		

};
#endif 
