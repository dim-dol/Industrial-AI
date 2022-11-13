

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int sum=0;

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
	sum += msg->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node4");
	ros::NodeHandle n;

	ros::Subscriber node4 = n.subscribe("topicC", 1000, chatterCallback);

	ros::Rate loop_rate(1);
	int count =1;
	int average;
	while (ros::ok())
	{

		average = sum/count;
		
		ROS_INFO("[ %d ] prefix average = %d \n",count,average);
		count ++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
