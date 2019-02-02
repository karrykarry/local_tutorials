#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
using namespace std;
#define MAX(x,y) ((x) > (y) ? (x) : (y))

class Vicon{
	private:
		ros::Subscriber sub_;
		ros::Subscriber yawsub_;
		ros::Publisher pub_;
		tf::TransformBroadcaster tfbroadcaster_;
		double yaw;
	public:
		Vicon(ros::NodeHandle n,ros::NodeHandle private_nh);
		void callback(const geometry_msgs::TransformStamped::Ptr msg);
		void yawcallback(const std_msgs::Float64::Ptr msg);
};

int main(int argc, char** argv){
	ros::init(argc, argv, "vicon_odom");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	Vicon vo(n,private_nh);

	ros::spin();

	return 0;  
}

Vicon::Vicon(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	yaw(0.0)
{
	sub_ = n.subscribe("/vicon_topic", 2, &Vicon::callback, this);
	// sub_ = n.subscribe("/yaw/icp", 2, &Vicon::yawcallback, this);
	// yawsub_ = n.subscribe("/yaw/icp_removed_human", 2, &Vicon::yawcallback, this);
	yawsub_ = n.subscribe("/yaw/proposed_method", 2, &Vicon::yawcallback, this);
	pub_ = n.advertise<std_msgs::Float64>("/yaw/ground_truth", 10);

	cout<<"vicon start"<<endl;
}

double yaw_max = 0.0;

void
Vicon::callback(const geometry_msgs::TransformStamped::Ptr msg){


	geometry_msgs::Quaternion q;
	q = msg->transform.rotation;

    double roll,pitch,yaw_;
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_);
	yaw = (double)yaw_;
	// cout <<"yaw:"<<yaw<<endl;
	std_msgs::Float64 pub_data;
	pub_data.data = (double)yaw;

	pub_.publish(pub_data);

	geometry_msgs::TransformStamped st;
	
	st = *msg;
	st.header.frame_id = "/map";
	st.child_frame_id = "/vicon";

	tfbroadcaster_.sendTransform(st);
}


void
Vicon::yawcallback(const std_msgs::Float64::Ptr msg){
	
	static bool start_flag = false;
	static double yaw_mean = 0.0;
	static double yaw_vari_sum = 0.0;
	static double yaw_sum = 0.0;
	static int yaw_cnt = 1;
	
	if(!start_flag){
		yaw_max =fabs(msg->data - yaw);
		start_flag=true;
	}
	yaw_max = MAX(yaw_max,fabs(msg->data - yaw));

	
	yaw_mean = (fabs(msg->data - yaw) + yaw_mean * (yaw_cnt-1)) / yaw_cnt;

	yaw_vari_sum += pow(fabs(msg->data - yaw),2); 
	double yaw_vari = (yaw_vari_sum / (yaw_cnt-1))  - pow(yaw_mean,2);

	cout<<"max"<<yaw_max<<endl;
	cout<<"yaw_mean"<<yaw_mean<<endl;
	cout<<"yaw_vari"<<yaw_vari<<endl;
	
	
	yaw_cnt++;

}
