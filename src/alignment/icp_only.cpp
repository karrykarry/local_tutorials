/*
 * icpのみによる位置合わせ
 *
 */
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


#include <tf/transform_broadcaster.h>

using namespace std;

class Icp{
	private:
	ros::Subscriber pc_sub;
	ros::Publisher buffer_pub;
	ros::Publisher src_pub;
	ros::Publisher out_pub;
	ros::Publisher pub;

	bool first_flag;
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr buffer_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr after_cloud;
	public:
	Icp(ros::NodeHandle n,ros::NodeHandle private_nh);
	void callback(const sensor_msgs::PointCloud2::Ptr msg);
	void matching(pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,ros::Time t);
	void calc_rpy(Eigen::Matrix4f a);

};


int main(int argc, char** argv){
	ros::init(argc, argv, "icp_only");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	Icp icp_(n,private_nh);

	ros::spin();

	return 0;  
}


Icp::Icp(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	first_flag(false),
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	buffer_cloud(new pcl::PointCloud<pcl::PointXYZI>),	//前の点群
	after_cloud(new pcl::PointCloud<pcl::PointXYZI>)		//icp後の点群
{
	pc_sub = n.subscribe("/velodyne_points", 2, &Icp::callback, this);
	// buffer_pub = n.advertise<sensor_msgs::PointCloud2>("/buffer", 10);
	// src_pub = n.advertise<sensor_msgs::PointCloud2>("/src", 10);
	// out_pub = n.advertise<sensor_msgs::PointCloud2>("/out", 10);
	pub = n.advertise<std_msgs::Float64>("/yaw/icp", 10);

	cout<<"icp start"<<endl;
}


void
Icp::callback(const sensor_msgs::PointCloud2::Ptr msg){

	static int cnt = 0;

	if(cnt%3==0){
		ros::Time t = msg->header.stamp;

		*buffer_cloud = *input_cloud;
		pcl::fromROSMsg (*msg, *input_cloud);
		if(!first_flag){
			cout<<"firsr"<<endl;
			pcl::fromROSMsg (*msg, *buffer_cloud);
			first_flag = true;
		}
		matching(buffer_cloud,input_cloud,t);
	}
	cnt++;

}

void
Icp::matching(pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,ros::Time t){
    
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setMaxCorrespondenceDistance(10.0);
	icp.setMaximumIterations(20);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);
    
	icp.setInputTarget(tgt_cloud);	//おおもと
	icp.setInputSource(src_cloud);
	icp.align(*after_cloud);
    
    
	Eigen::Matrix4f a;
	a =  icp.getFinalTransformation();
    
	calc_rpy(a);


	// sensor_msgs::PointCloud2 buffer_pc,src_pc,out_pc;
    //
	// pcl::toROSMsg(*tgt_cloud,buffer_pc);
	// pcl::toROSMsg(*src_cloud,src_pc);
	// pcl::toROSMsg(*after_cloud,out_pc);
	//
	// buffer_pc.header.frame_id  = "velodyne";	
	// buffer_pc.header.stamp  = t;
	// buffer_pub.publish(buffer_pc);
    //
    //
	// src_pc.header.frame_id  = "velodyne";	
	// src_pc.header.stamp  = t;
	// src_pub.publish(src_pc);
    //
	// out_pc.header.frame_id  = "velodyne";	
	// out_pc.header.stamp  = t;
	// out_pub.publish(out_pc);


	// buffer_cloud = after_cloud;
	// local_tutorials
}


void 
Icp::calc_rpy(Eigen::Matrix4f a){

	static double yaw_ = 0;

	double roll, pitch, yaw;//角度etc
	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

	mat_l.getRPY(roll, pitch, yaw, 1);


	cout<<"roll:"<<roll<<",pitch:"<<pitch<<",yaw:"<<yaw<<endl;
	cout<<"x:"<<a(0, 3)<<",y:"<<a(1, 3)<<",z:"<<a(2, 3)<<endl;
	cout<<"yaw_"<<yaw_<<endl;

	std_msgs::Float64 data;

	yaw_ += yaw;

	data.data = yaw_;

	pub.publish(data);
}
