#ifndef _MAP_MATCHING_TF_HPP_
#define _MAP_MATCHING_TF_HPP_
#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>

#include<local_tutorials/OdoUpdate.h>

using namespace std;


class Map_tf{

	private:	
		ros::Publisher debug_pub;
		ros::ServiceClient client;	
		
		tf::TransformBroadcaster br;
		tf::Transform broad_transform;
		tf::TransformListener listener;
		tf::StampedTransform buffer_transform;
		
		local_tutorials::OdoUpdate srv;
		void debug_publisher();
		void odo_request(nav_msgs::Odometry& ndt);
	
	public:
		Map_tf(ros::NodeHandle n,ros::NodeHandle private_nh_);
		void listen_tf(string from_id, string to_id);

};

#endif


