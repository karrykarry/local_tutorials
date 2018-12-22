#ifndef _MCL_FUNC_HPP_
#define _MCL_FUNC_HPP_
#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>


class MCL;

class MCL_F{

	private:
		MCL* m_MCL;
		ros::NodeHandle n;
		ros::NodeHandle priv_nh;

		typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;

		message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
		message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
		message_filters::Synchronizer<SyncPolicy> sync;
	
		ros::Time last_time;
		double dist,yaw;
		bool start_flag;

		double dt_calc(ros::Time current_time);

	public:
		MCL_F(MCL* mp_MCL);
	
		void syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu);
		void move_model(vector<Particle>& pf_cloud);

};

#endif

