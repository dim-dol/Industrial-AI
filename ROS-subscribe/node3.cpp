

#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int64_t A=1,B=1;

void chatterCallback_A(const std_msgs::Int64::ConstPtr& msg)
{
	A = msg->data;
}

void chatterCallback_B(const std_msgs::Int64::ConstPtr& msg)
{
	B = msg->data;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "node3");
	ros::NodeHandle n;

	ros::Subscriber node3_1 = n.subscribe("topicA", 1000, chatterCallback_A);
	ros::Subscriber node3_2 = n.subscribe("topicB", 1000, chatterCallback_B);
	ros::Publisher node3_3 = n.advertise<std_msgs::Int64>("topicC",1000);

	ros::Rate loop_rate(1);
	int count =0;
	while (ros::ok())
	{

		std_msgs::Int64 num3;

		num3.data = (A + B)/2;


		ROS_INFO("[%d] average of A(%d), B(%d) = %d\n",count,A,B,num3.data);
				
		node3_3.publish(num3);
		count ++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}






