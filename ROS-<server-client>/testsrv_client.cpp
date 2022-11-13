/*
*
* 2021254018 김원우
*
* 2021.11.23
*
* 사용자 입력, 반지름 값 생성, 서버로 데이터 전달, publish
*
*/
#include "ros/ros.h"
#include "testpkg/testsrv.h"
#include "testpkg/testmsg.h"
#include <cstdlib>
#include <vector>
#include <time.h>

std::vector<long int> storedVector;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calc_area_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<testpkg::testsrv>("calc_area");
	ros::Publisher chatter_pub = n.advertise<testpkg::testmsg>("custommsg", 1);
	ros::Rate loop_rate(1);
	int count = 0;

	testpkg::testsrv srv;
	
	if (argc != 2)
	{
		srv.request.num = 5;
	}
	else
		srv.request.num =atoll(argv[1]);
	
	for(int i=0;i<srv.request.num;i++)
	{
		storedVector.push_back(i);
	}

	while(ros::ok())
	{
		testpkg::testmsg pub_data;
		pub_data.header.seq = count;
		pub_data.header.stamp = ros::Time::now();

		srand(time(NULL));

		for(int i=0;i<srv.request.num;i++)
		{
			storedVector.push_back(rand()%100);
			storedVector.erase(storedVector.begin());	
		}
		pub_data.radius_array = storedVector;
		srv.request.radius_array = pub_data.radius_array;
		srv.request.flag = count;
		if (client.call(srv))
		{
			pub_data.area_array = srv.response.area_array;
		}
		else
		{
			ROS_ERROR("Failed to call service");
			return 1;
		}
		chatter_pub.publish(pub_data);
		std::cout << "["<< count <<"] pub!" << std::endl;
		count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
