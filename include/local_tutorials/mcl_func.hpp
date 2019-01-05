#ifndef _MCL_FUNC_HPP_
#define _MCL_FUNC_HPP_
#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include<random>


class MCL;

class MCL_F{

	private:
		MCL* m_MCL;
		ros::NodeHandle n;
		ros::NodeHandle priv_nh;
		ros::Publisher grid_pub;
		ros::Subscriber map_sub;
		ros::Subscriber lidar_sub;

		typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;

		message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
		message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
		message_filters::Synchronizer<SyncPolicy> sync;
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud;

		ros::Time last_time;
		nav_msgs::OccupancyGrid map_grid;
		nav_msgs::OccupancyGrid local_map;		//sensor_map
		double dist,yaw;
		bool start_flag;

		int cell_filter;
		float local_width,local_height,R;
		double min_x,min_y;

		vector<double> weight_array;
		double weight_sum;

		double dt_calc(ros::Time current_time);
		void create_obstacle_map(pcl::PointCloud<pcl::PointXYZI>::Ptr clouds,nav_msgs::OccupancyGrid *local_map);

	public:
		MCL_F(MCL* mp_MCL);

		void syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu);
		void gridmapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg);
		void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &input);
		void move_model(vector<Particle>& pf_clouds);
		double measurement_model(double x,double y,double yaw);

		void resample(vector<Particle> pf_cloud,vector<Particle>& new_pf_cloud);
		void reset();
		void say();

};

#endif

