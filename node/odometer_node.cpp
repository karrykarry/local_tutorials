/* odometer.cpp
 *
 * 2018.12.21
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include "odometer.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometer");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-----odom + imu----" <<endl;

	Dead_rec dead_rec(n,priv_nh);

	ros::spin();

    return 0;
}

       
