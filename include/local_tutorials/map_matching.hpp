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
#include<nav_msgs/Odometry.h>
#include<local_tutorials/OdoUpdate.h>

using namespace std;


class Matcher{

	private:
		ros::Publisher pc_pub;
		ros::Publisher map_pub;
		ros::Publisher lcl_pub;
		ros::Subscriber pc_sub;

		ros::ServiceServer service;

		double map_limit,x_now,y_now;
		pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr low_lidar_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_cloud;
    	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;//default value

		sensor_msgs::PointCloud2 vis_pc;
		
		bool odo_response(local_tutorials::OdoUpdate::Request  &req,
        		 local_tutorials::OdoUpdate::Response &res);
		void local_map(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);
	
	public:
		Matcher(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void lidarcallback(const sensor_msgs::PointCloud2::Ptr msg);
		void pc_publisher(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,string frame_id);
		void map_read(string filename, double voxel_size);
		void aligner_ndt(double& roll, double& pitch, double& yaw, double& x, double& y, double& z,double voxel_size);
		void lcl_publisher(double roll, double pitch, double yaw, double x, double y, double z);
};

#endif

