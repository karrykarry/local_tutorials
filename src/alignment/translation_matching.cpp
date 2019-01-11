#include"translation_matching.hpp"


Icp::Icp(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	buffer_cloud(new pcl::PointCloud<pcl::PointXYZI>),	//前の点群
	after_cloud(new pcl::PointCloud<pcl::PointXYZI>)		//icp後の点群
{
	// pc_sub = n.subscribe("/velodyne_points", 2, &Icp::callback, this);
	pc_sub = n.subscribe("/voxelGrid/static", 2, &Icp::callback, this);
	// buffer_pub = n.advertise<sensor_msgs::PointCloud2>("/buffer", 10);
	icp_pub = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/using_icp", 10);
	// out_pub = n.advertise<sensor_msgs::PointCloud2>("/out", 10);
	pub = n.advertise<std_msgs::Float64>("/yaw/icp", 10);
	odom_pub = n.advertise<nav_msgs::Odometry>("/kari_trajectory", 10);
	
	rota_pose.header.frame_id = "/kari_transform";
	rota_pose.pose.position.x = 0.0;
	rota_pose.pose.position.y = 0.0;
	rota_pose.pose.position.z = 0.0;
	rota_pose.pose.orientation.x = 0.0;
	rota_pose.pose.orientation.y = 0.0;
	rota_pose.pose.orientation.z = 0.0;
	rota_pose.pose.orientation.w = 1.0;

	odom.header.frame_id = "/map";
	odom.child_frame_id = "/kari_transform";


	buffer_pose_ = vector<double> (3,0.0);

}


void
Icp::callback(const sensor_msgs::PointCloud2::Ptr msg){

	static int cnt = 0;
	static double dist = 0;
	static bool first_flag = false;
	vector<double> dpose_(3,0.0); 

	// if(cnt%3==0){
	// 	ros::Time t = msg->header.stamp;
    //
	// 	pcl::fromROSMsg (*msg, *input_cloud);
	if(!first_flag){
		cout<<"first"<<endl;
	 	pcl::fromROSMsg (*msg, *input_cloud);
		*buffer_cloud = *input_cloud;
		geometry_msgs::TransformStamped ts;
		
		ts.header.frame_id = "/matching_base_link";
		ts.child_frame_id = "/kari_transform";
		ts.transform.translation.x = 0.0;
		ts.transform.translation.y = 0.0;
		ts.transform.translation.z = 0.0;
		ts.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
		tfbroadcaster_.sendTransform(ts);
		first_flag = true;
	}
	// 	matching(buffer_cloud,input_cloud,t);
	// 	*buffer_cloud = *input_cloud;
	// }
	cnt++;

	map2ienage(dpose_,msg->header.stamp);
	map2transform(msg->header.stamp);
	
	dist = diffdist(dpose_);

	if(dist>2.0){
		cout<<"\x1b[42m 2mきたよ \x1b[m\r"<<endl;
		for(int i=0;i<3;i++){ buffer_pose_[i] = dpose_[i];
	 	// matching(buffer_cloud,input_cloud,msg->header.stamp);
	 	*buffer_cloud = *input_cloud;
		pub_pc(input_cloud,msg->header.stamp);
		}
	}
	
	
	broadTF_kari(rota_pose,dpose_[0],dpose_[1],dpose_[2],msg->header.stamp);	

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


void
Icp::map2ienage(vector<double>& dpose,ros::Time t){		//角度を気にせず移動量を出すため

	try{
		
		tflistener_.lookupTransform("/ienaga", "/map",  
				t, buffer_transform_pose);
		//mapから見ているから反転
		dpose[0] = buffer_transform_pose.getOrigin().x() * (-1);
		dpose[1] = buffer_transform_pose.getOrigin().y() * (-1);
		dpose[2] = buffer_transform_pose.getOrigin().z() * (-1);	
	
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

void
Icp::map2transform(ros::Time t){
	try{
		
		tflistener_.lookupTransform("/transform", "/map",  
				t, buffer_transform);
		
		tf::Quaternion quatDiff = getQuaternionInverse(buffer_transform.getRotation());

		rota_pose.pose.orientation.x = quatDiff.x();
		rota_pose.pose.orientation.y = quatDiff.y();
		rota_pose.pose.orientation.z = quatDiff.z();
		rota_pose.pose.orientation.w = quatDiff.w();

	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

}



double
Icp::diffdist(vector<double> dpose_){
	double dist_;

	dist_ =sqrt( pow(dpose_[0]-buffer_pose_[0],2)+
			pow(dpose_[1]-buffer_pose_[1],2)+pow(dpose_[1]-buffer_pose_[1],2));

	return dist_;
}


void
Icp::pub_pc(pcl::PointCloud<pcl::PointXYZI>::Ptr pub_cloud,ros::Time t){
	sensor_msgs::PointCloud2 pub_pc_;
    
	pcl::toROSMsg(*pub_cloud,pub_pc_);
	
	pub_pc_.header.frame_id  = "kari_transform";	
	pub_pc_.header.stamp  = t;
	icp_pub.publish(pub_pc_);

}



void
Icp::broadTF_kari(geometry_msgs::PoseStamped pose,double xx ,double yy, double zz, ros::Time t){

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(xx, yy, zz));
	tf.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
	tfbroadcaster_.sendTransform(tf::StampedTransform(tf, t, std::string("/matching_base_link"), std::string("/kari_transform") ));
	odom.pose.pose.position.x = xx;
	odom.pose.pose.position.y = yy;
	odom.pose.pose.position.z = zz;

	odom.pose.pose.orientation = pose.pose.orientation;

	odom_pub.publish(odom);

}

tf::Quaternion 
Icp::getQuaternionInverse(tf::Quaternion quat){
	tf::Matrix3x3 mat;
	double roll, pitch, yaw;
	mat.setRotation(quat);
	mat.getRPY(roll, pitch, yaw);
	tf::Quaternion quatInverse = tf::createQuaternionFromRPY(-1.0f*roll, -1.0f*pitch, -1.0f*yaw);

	return quatInverse;
}

