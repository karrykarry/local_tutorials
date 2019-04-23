#include"odometer.hpp"

Dead_rec::Dead_rec(ros::NodeHandle n,ros::NodeHandle priv_nh):
	odom_sub(n,"/odom",100),imu_sub(n,"/imu/data",100),
	sync(SyncPolicy(10),odom_sub,imu_sub),
	x(0.0),y(0.0),yaw(0.0),
	flag(true)
{
	sync.registerCallback(boost::bind(&Dead_rec::syncMsgsCB, this, _1, _2));

	odom_pub = n.advertise<nav_msgs::Odometry>("/odometer",100);

	priv_nh.getParam("parent_frame",parent_frame);
	priv_nh.getParam("child_frame",child_frame);
	
	lcl.header.frame_id = "/map";
	lcl.child_frame_id = "/base_link";
}


void
Dead_rec::syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){
	double odom_vel,dyaw,dt;
	geometry_msgs::Quaternion odom_quat;

	odom_vel = odom->twist.twist.linear.x;
 	dyaw = imu->angular_velocity.z;
	dt = dt_calc(imu->header.stamp);

	double dist = odom_vel * dt; 
	
	yaw += dyaw * dt; 

	while(yaw > M_PI) yaw -= 2*M_PI;
	while(yaw < -M_PI) yaw += 2*M_PI;

	x += dist * cos(yaw);// * cos(pitch);
	y += dist * sin(yaw);// * cos(pitch);
	odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	pub(imu->header.stamp,odom_quat);
}

double
Dead_rec::dt_calc(ros::Time current_time){
	
	if(flag){ 
		last_time = current_time;
		flag = false;		
	}

	double last_accurate,current_accurate;
	last_accurate = (double)last_time.nsec*1.0e-9 + last_time.sec;
	current_accurate = (double)current_time.nsec*1.0e-9 + current_time.sec;
	double dt = current_accurate - last_accurate;
	last_time = current_time;

	return dt;
}

void
Dead_rec::pub(ros::Time current_time,geometry_msgs::Quaternion quat){

	lcl.header.stamp = current_time;
	lcl.pose.pose.position.x = x;
	lcl.pose.pose.position.y = y;
	lcl.pose.pose.position.z = 0.0;
	lcl.pose.pose.orientation = quat;

	odom_pub.publish(lcl);

	transform.setOrigin( tf::Vector3( x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, yaw);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, current_time, parent_frame, child_frame));
}
