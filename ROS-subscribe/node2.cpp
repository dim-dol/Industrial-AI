

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void chatterCallback(const std_msgs::Int64::ConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node2");
	ros::NodeHandle n;

	ros::Publisher node2 = n.advertise<std_msgs::Int64>("topicB",1000);

	ros::Rate loop_rate(1);
	int count =0;
	while (ros::ok())
	{
		std_msgs::Int64 num2;

		srand(time(NULL));
		num2.data = rand()%10 + 11; // 11 ~ 20
		
		ROS_INFO("[%d] random number B = %d\n",count,num2.data);
				
		node2.publish(num2);
		count ++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}






