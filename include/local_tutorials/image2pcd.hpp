#ifndef _MAP2IMAGE_HPP_
#define _MAP2IMAGE_HPP_
#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/approximate_voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/ros/conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/xfeatures2d.hpp>


using namespace std;

#define MAX_NUM 255		// 0~255 の値

class Image2pcd{

	private:
		ros::Publisher pc_pub_;
		
		int map_limit;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cloud;
		pcl::PointXYZ info2pc(float r, float theta, float phi);   
	
	public:
		Image2pcd(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void create_pcd(string filename);
};

#endif


