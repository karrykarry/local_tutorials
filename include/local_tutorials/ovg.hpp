#ifndef _OVG_HPP_
#define _OVG_HPP_


#include <ros/ros.h>
#include <iostream>
#include <omp.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <original_msgs/pointsWithPose.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

class OVG
{
	private:
		vector<sensor_msgs::PointCloud> pc_vec;	// velodyneのglobal座標系に変換したデータ格納変数
		vector<vector<int> >is_shot_pc;			// 各voxel内の点群存在判定用ベクトル
		vector<int> is_shot_nowPC;				// 現在のvoxel内点群存在判定用
		sensor_msgs::PointCloud2 pc_cluster;	// 確認用
		geometry_msgs::PoseStamped init_pose;
		ros::Time sub_time_;
		tf::TransformListener tflistener;
		tf::TransformBroadcaster tfbroadcaster_;
		tf::Transform tf_;
		ros::Subscriber pc_sub;
        ros::Publisher static_voxel_pub;
        ros::Publisher free_voxel_pub;
		ros::Publisher prob_voxel_pub;
		ros::Publisher occlusion_voxel_pub_;
        ros::Publisher pc_pub;
		ros::Publisher pc_debug_1;
		ros::Publisher pc_debug_2;
		ros::Publisher pc_debug_3;
		ros::Publisher pc_debug_4;
		ros::Publisher pc_debug_5;
		ros::Publisher pc_debug_6;
        ros::Publisher static_points_pub;
		ros::Publisher pcpose_pub;
		struct timeval start, finish;
		double dt;
		/*---- ROS static parameter ----*/
		double max_x_range;
		double max_y_range;
		double max_z_range;
		double voxelSize;
		int skip;
		int step;
		double prob_threshold;
		int save_raw_flag;
		int debug_pub_flag;

		/*---- PCL static parameter ----*/
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_CoG;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_static;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_free;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_probability;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_normal;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr voxel_occlusion;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_idx;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_static;

	public:
		OVG(ros::NodeHandle n,ros::NodeHandle private_nh_);
		void setVoxelGrid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);
		void pcCallback(const sensor_msgs::PointCloud2::Ptr msg);
		void pcHandle(void);
		void transformOdom2velodyne(vector<pcl::PointCloud<pcl::PointXYZINormal> >& pcl_vec, ros::Time t);
		void calc_pcPoint(vector<pcl::PointCloud<pcl::PointXYZINormal> > pcl_vec, vector<vector<int> >& is_shot);
		void calc_nowPC_isShot(const sensor_msgs::PointCloud2::Ptr cloud_raw);
		void supplement(int iteration);
		bool getPose(geometry_msgs::Pose& msg, ros::Time t);
		void broad_tf(original_msgs::pointsWithPose msg);
		void save_rawData(const sensor_msgs::PointCloud2::Ptr cloud_raw);
};




#endif


// voxel_CoG (new pcl::PointCloud<pcl::PointXYZINormal>);			// ボクセルグリッド表現用
// voxel_static (new pcl::PointCloud<pcl::PointXYZINormal>);		// 占有空間
// voxel_free (new pcl::PointCloud<pcl::PointXYZINormal>);			// 非占有空間
// voxel_probability (new pcl::PointCloud<pcl::PointXYZINormal>);	// 占有率確率空間
// voxel_normal (new pcl::PointCloud<pcl::PointXYZINormal>);				// ボクセルの平均法線格納
// voxel_occlusion (new pcl::PointCloud<pcl::PointXYZINormal>);		// 未知占有可能性空間
// cloud_idx (new pcl::PointCloud<pcl::PointXYZINormal>);			// 占有確率点群
// cloud_static (new pcl::PointCloud<pcl::PointXYZINormal>);		// 占有点群
// cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);		// 色付け用


