/*
 *
 */
#include"map_matching_tf.hpp"

Map_tf::Map_tf(ros::NodeHandle n,ros::NodeHandle private_nh_) 
{
	debug_pub = n.advertise<nav_msgs::Odometry>("/odometry/debug", 10);
	client = n.serviceClient<local_tutorials::OdoUpdate>("odometry/ndt");
}



void
Map_tf::listen_tf(string from_id, string to_id){

	try{
		ros::Time time_now = ros::Time(0);
		listener.waitForTransform(from_id, to_id, time_now, ros::Duration(0.5));
		
		listener.lookupTransform(from_id, to_id,  time_now, buffer_transform);
		debug_publisher();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


void
Map_tf::debug_publisher(){

	tf::Quaternion quat = tf::createQuaternionFromRPY(0,0, 
			tf::getYaw(buffer_transform.getRotation()));
	
	geometry_msgs::Quaternion geometry_quat;
	quaternionTFToMsg(quat, geometry_quat);

	nav_msgs::Odometry tf_debug;
		
	tf_debug.pose.pose.position.x = buffer_transform.getOrigin().x();
	tf_debug.pose.pose.position.y = buffer_transform.getOrigin().y();
	tf_debug.pose.pose.position.z = buffer_transform.getOrigin().z();
	tf_debug.pose.pose.orientation = geometry_quat;	

	odo_request(tf_debug);

	tf_debug.header.stamp = ros::Time::now();
	tf_debug.header.frame_id = "/map";

	debug_pub.publish(tf_debug);
}

void
Map_tf::odo_request(nav_msgs::Odometry& ndt){
	srv.request.before = ndt.pose.pose;

	if (client.call(srv))
	{
		cout<<"odometry --> NDT Matching"<<endl; 
		ndt.pose.pose = srv.response.after;

	}
	else
	{
		cout<<"\x1b[31m"<<"-----No Update-----"<<"\x1b[m\r"<<endl;
		return;
	}
}
