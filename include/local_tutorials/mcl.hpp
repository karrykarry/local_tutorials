#ifndef _MCL_HPP_
#define _MCL_HPP_
#include<iostream>
#include<ros/ros.h>
// #include <dynamic_reconfigure/server.h>
// #include <local_tutorials/ParamConfig.h>

using namespace std;

struct Particle{
	double x;
	double y;
	double yaw;
	double weight;
};


class MCL{

	private:
		Particle pf_state();
	public:
		MCL(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void pub();
};

#endif
