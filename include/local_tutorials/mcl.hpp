#ifndef _MCL_HPP_
#define _MCL_HPP_
#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<dynamic_reconfigure/server.h>
#include<local_tutorials/mcl_paramConfig.h>

using namespace std;

struct Particle{
	double x;
	double y;
	double yaw;
	double weight;
};


class MCL{

	private:
		dynamic_reconfigure::Server<local_tutorials::mcl_paramConfig> server;
		ros::Publisher pf_pub;
		geometry_msgs::PoseArray pf_array;

		int pf_num;

		Particle pf_state();
		geometry_msgs::Pose pf2geo(Particle pf);

	public:
		MCL(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void pub();
		void paramcallback(local_tutorials::mcl_paramConfig &config, uint32_t level);
};

#endif
