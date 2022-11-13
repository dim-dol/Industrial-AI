/*
*
* 2021254018 김원우
*
* 2021.11.23
*
* 원의 넓이 값 계산
*
*/
#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "testpkg/testsrv.h"
#include <math.h>
#include <vector>

std::vector<double> storedVector;

bool calc_area(testpkg::testsrv::Request &req, testpkg::testsrv::Response &res)
{
	int num = req.num;
	int A;
	int flag = req.flag;

	if(flag==0)
	{
		for(int i=0; i<req.num;i++)
			storedVector.push_back(i);
	}
	std::cout << "radius : ";

	for(int i=0;i<req.num;i++)
	{
		std::cout << " " << req.radius_array.at(i);
 
		A = req.radius_array[i];
		
		storedVector.push_back(A*A*M_PI);
		storedVector.erase(storedVector.begin());
		res.area_array =storedVector;
	}
	std::cout << std::endl;

	return true;
}

int main(int argc, char **argv)
{
	for(int i=0; i<8;i++)
	{
		storedVector.push_back(i);
	}

	ros::init(argc, argv, "calc_area_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("calc_area", calc_area);
	ROS_INFO("Ready to calc area.");
	ros::spin();

	return 0;
}
