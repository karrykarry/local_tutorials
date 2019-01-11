#ifndef _TRANSLATION_MATCHING_HPP_
#define _TRANSLATION_MATCHING_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class Icp{
	private:
	ros::Subscriber pc_sub;
	ros::Publisher buffer_pub;
	ros::Publisher icp_pub;
	ros::Publisher out_pub;
	ros::Publisher pub;
	ros::Publisher odom_pub;
	tf::TransformListener tflistener_;
	tf::TransformBroadcaster tfbroadcaster_;

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr buffer_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr after_cloud;

	vector<double> buffer_pose_;
	tf::StampedTransform buffer_transform_pose;
	tf::StampedTransform buffer_transform;
	tf::StampedTransform base_link_transform;
	geometry_msgs::PoseStamped rota_pose;
	nav_msgs::Odometry odom;

	public:
	Icp(ros::NodeHandle n,ros::NodeHandle private_nh);
	void callback(const sensor_msgs::PointCloud2::Ptr msg);
	void matching(pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,ros::Time t);
	void calc_rpy(Eigen::Matrix4f a);


	void map2ienage(vector<double>& dpose,ros::Time t);
	void map2transform(ros::Time t);
	double diffdist(vector<double> dpose_);

	void pub_pc(pcl::PointCloud<pcl::PointXYZI>::Ptr pub_cloud,ros::Time t);
	
	void broadTF_kari(geometry_msgs::PoseStamped pose,double xx ,double yy, double zz, ros::Time t);
	tf::Quaternion getQuaternionInverse(tf::Quaternion quat);

};

#endif

