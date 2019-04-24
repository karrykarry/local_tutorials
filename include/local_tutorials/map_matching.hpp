#ifndef _MAP_MATCHING_HPP_
#define _MAP_MATCHING_HPP_
#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/registration/ndt.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/approximate_voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/ros/conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<sensor_msgs/PointCloud2.h>
#include<tf/transform_broadcaster.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<local_tutorials/OdoUpdate.h>

using namespace std;


class Matcher{

	private:
		ros::Publisher pc_pub;
		ros::Publisher map_pub;

		ros::Subscriber pc_sub;
		ros::ServiceServer service;

		double map_limit;
		pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr low_lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_cloud;
    	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;//default value
		
		double voxel_size;
		
		bool odo_response(local_tutorials::OdoUpdate::Request  &req,
        		 local_tutorials::OdoUpdate::Response &res);
		geometry_msgs::Pose aligner_ndt(geometry_msgs::Pose before_pose);
		void local_map(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,double x_now,double y_now);
		
		void pc_publisher(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,string frame_id);
	
	public:
		Matcher(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void map_read(string filename);
		void lidarcallback(const sensor_msgs::PointCloud2::Ptr msg);
};

#endif

