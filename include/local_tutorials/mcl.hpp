#ifndef _MCL_HPP_
#define _MCL_HPP_
#include<iostream>
#include<ros/ros.h>
#include<omp.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<dynamic_reconfigure/server.h>
#include<local_tutorials/mcl_paramConfig.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>

using namespace std;

struct Particle{
	double x;
	double y;
	double yaw;
	double weight;
};

class MCL_F;

class MCL{

	private:
		dynamic_reconfigure::Server<local_tutorials::mcl_paramConfig> server;
		ros::Publisher pf_pub;
		ros::Publisher lcl_pub;
		geometry_msgs::PoseArray pf_array;
		nav_msgs::Odometry lcl_pf;

		tf::TransformBroadcaster br;
		tf::Transform transform; 

		int pf_num;

		vector<Particle> pf_cloud;
		vector<Particle> new_pf_cloud;

		void init_set();
		geometry_msgs::Pose pf2geo(Particle pf);
		void lcl_pub_(double x, double y,double yaw);

		MCL_F* m_MCL_F; 

	public:
		MCL(ros::NodeHandle n,ros::NodeHandle priv_nh);
		bool start_fg;
		void pub();
		void paramcallback(local_tutorials::mcl_paramConfig &config, uint32_t level);
};

#endif
