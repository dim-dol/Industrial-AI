

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void chatterCallback(const std_msgs::Int64::ConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node1");
	ros::NodeHandle n;

	ros::Publisher node1 = n.advertise<std_msgs::Int64>("topicA",1000);

	ros::Rate loop_rate(1);
	int count =0;
	while (ros::ok())
	{
		std_msgs::Int64 num1;

		srand(time(NULL));
		num1.data = rand()%10 + 1; // 11 ~ 20

		ROS_INFO("[%d] random number A = %d\n",count, num1.data);
				
		node1.publish(num1);
		count ++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
