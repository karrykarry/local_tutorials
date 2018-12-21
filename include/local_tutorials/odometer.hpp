#ifndef _ODOMETER_HPP_
#define _ODOMETER_HPP_

#include<ros/ros.h>
#include<iostream>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>



using namespace std;

class Dead_rec{

	private:
		typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;

		message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
		message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
		message_filters::Synchronizer<SyncPolicy> sync;

		ros::Publisher odom_pub;
		nav_msgs::Odometry lcl;
		tf::TransformBroadcaster br;
		tf::Transform transform; 
		
		ros::Time last_time;
		double x,y,yaw;
		bool flag;

		double dt_calc(ros::Time current_time);
		void pub(ros::Time current_time,geometry_msgs::Quaternion quat);
	public:
		Dead_rec(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu);

};

#endif
