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

class Map2image{

	private:
		ros::Publisher map_pub;
		ros::Publisher local_map_pub;
		ros::Publisher image_pub;
		
		cv::Mat based_image;

		int width, height;
		int map_limit;
		string pre_path;
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_cloud;

		void visible_map(pcl::PointCloud<pcl::PointXYZI>::Ptr clouds, cv::Mat &based_image);
		float distance(float x, float y, float z);
	public:
		Map2image(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void map_read(string filename);
		void pcd2image();
};

#endif


