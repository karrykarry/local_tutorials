#ifndef _MAP_MATCHER_HPP_
#define _MAP_MATCHER_HPP_
#include<iostream>
#include<ros/ros.h>
#include<vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<sensor_msgs/PointCloud2.h>

using namespace std;


class Matcher{

	private:
		ros::Publisher icp_pub;
		ros::Publisher map_pub;
		ros::Subscriber pc_sub;
		pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr limit_lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;

		sensor_msgs::PointCloud2 vis_pc;
		
		double map_limit,x_now,y_now;
	
	public:
		Matcher(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void lidarcallback(const sensor_msgs::PointCloud2::Ptr msg);
		void pc_publisher();
		void map_read(string filename, double size);
};

#endif

