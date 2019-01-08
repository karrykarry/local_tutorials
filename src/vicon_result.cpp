#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class Vicon{
	private:
		ros::Subscriber sub_;
		ros::Publisher pub_;
		tf::TransformBroadcaster tfbroadcaster_;
	public:
		Vicon(ros::NodeHandle n,ros::NodeHandle private_nh);
		void callback(const geometry_msgs::TransformStamped::Ptr msg);
};

int main(int argc, char** argv){
	ros::init(argc, argv, "vicon_odom");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	Vicon vo(n,private_nh);

	ros::spin();

	return 0;  
}

Vicon::Vicon(ros::NodeHandle n,ros::NodeHandle private_nh_)
{
	sub_ = n.subscribe("/vicon_topic", 2, &Vicon::callback, this);
	pub_ = n.advertise<std_msgs::Float64>("/yaw/ground_truth", 10);

	cout<<"vicon start"<<endl;
}

void
Vicon::callback(const geometry_msgs::TransformStamped::Ptr msg){

	geometry_msgs::Quaternion q;
	q = msg->transform.rotation;

    double roll,pitch,yaw;
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	cout <<"yaw:"<<yaw<<endl;
	
	std_msgs::Float64 pub_data;
	pub_data.data = (double)yaw;

	pub_.publish(pub_data);

	geometry_msgs::TransformStamped st;
	
	st = *msg;
	st.header.frame_id = "/map";
	st.child_frame_id = "/vicon";

	tfbroadcaster_.sendTransform(st);
}


