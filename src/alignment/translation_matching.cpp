#include"translation_matching.hpp"


Icp::Icp(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	yaw_judge(0.0),
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	input_cloud_(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	buffer_cloud(new pcl::PointCloud<pcl::PointXYZI>),	//前の点群
	after_cloud(new pcl::PointCloud<pcl::PointXYZI>)		//icp後の点群
{
	// pc_sub = n.subscribe("/velodyne_points", 2, &Icp::callback, this);
	// pc_sub = n.subscribe("/voxelGrid/static", 2, &Icp::callback, this);
	pc_sub = n.subscribe("/PointCloud/static", 2, &Icp::callback, this);
	// buffer_pub = n.advertise<sensor_msgs::PointCloud2>("/buffer", 10);
	icp_pub = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/static_", 10);
	// _pub = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/using_icp_", 10);
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
	buffer_angle_ = vector<double> (3,0.0);

}


void
Icp::callback(const sensor_msgs::PointCloud2::Ptr msg){

	static double dist = 0;
	static bool first_flag = false;
	vector<double> dpose_(3,0.0); 
	vector<double> dangle_(3,0.0); 

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
		pub_pc(input_cloud,msg->header.stamp);
		first_flag = true;
	}




	map2ienage(dpose_,msg->header.stamp);
	map2transform(dangle_,msg->header.stamp);

	cout<<"\x1b[42m"<<dangle_[0]<<","<<dangle_[1]<<","<<dangle_[2]<<","<< "\x1b[m\r"<<endl;

	dist = diffdist(dpose_);
	if(dist>2.0){
		// sensor_msgs::PointCloud2 change_point2;
		//
		// sub_t = msg->header.stamp;
		// sensor_msgs::convertPointCloud2ToPointCloud(*msg, buffer_point);
		// change_point2 = listen_tf(buffer_point);		
		// pcl::fromROSMsg (change_point2, *input_cloud);
		// if(fabs(dangle_[2]-buffer_angle_[2])<0.1)pub_pc(input_cloud,msg->header.stamp);


		pcl::fromROSMsg (*msg, *input_cloud);
		matching(buffer_cloud,input_cloud,dpose_,dangle_);
		// pcl::fromROSMsg (*msg, *input_cloud);
		// cout<<"\x1b[42m 1.5mきたよ \x1b[m\r"<<endl;
	if(fabs(dangle_[2]-buffer_angle_[2])<0.1)pub_pc(input_cloud,msg->header.stamp);
		for(int i=0;i<3;i++){ 
			buffer_pose_[i] = dpose_[i];
			buffer_angle_[i] = dangle_[i];	
		}
		*buffer_cloud = *input_cloud;
	// pcl::fromROSMsg (*msg, *input_cloud_);
	 // broadTF_kari(rota_pose,dpose_[0],dpose_[1],dpose_[2],msg->header.stamp);	
	}
	 broadTF_kari(rota_pose,dpose_[0],dpose_[1],dpose_[2],msg->header.stamp);	


}


void
Icp::matching(pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,vector<double> dpose_,vector<double> dangle_){

	 // MatrixXf I = MatrixXf::Identity(3,3);//単位行列
	 Eigen::Matrix3f rot;
	 rot = Eigen::AngleAxisf(
	 		dangle_[0]-buffer_angle_[0], Eigen::Vector3f::UnitX()) * 
	 	Eigen::AngleAxisf(dangle_[1]-buffer_angle_[1], Eigen::Vector3f::UnitY()) * 
	 	Eigen::AngleAxisf(dangle_[2]-buffer_angle_[2], Eigen::Vector3f::UnitZ());   
    
	 Eigen::Translation3f init_translation (dpose_[0]-buffer_pose_[0], dpose_[1]-buffer_pose_[1], dpose_[1]-buffer_pose_[1]);
    
	 Eigen::Matrix4f transform = (rot * init_translation).matrix ();
    
    

	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setMaxCorrespondenceDistance(0.5);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);
    
	icp.setInputTarget(tgt_cloud);	//おおもと
	icp.setInputSource(src_cloud);
	icp.align(*after_cloud,transform);
    
    
	Eigen::Matrix4f a;
	a =  icp.getFinalTransformation();
    
	calc_rpy(a);
}


void 
Icp::calc_rpy(Eigen::Matrix4f a){

	static double yaw_ = 0;
	static double x_sum = 0;
	static double y_sum = 0;
	static double z_sum = 0;

	double roll, pitch, yaw;//角度etc
	double x_,y_,z_;
	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

	// mat_l.getRPY(roll, pitch, yaw, 1);
    //
	// tf::Quaternion quaternion=tf::createQuaternionFromRPY(roll,pitch,yaw);
    //
	// geometry_msgs::Quaternion quat_Msg;
	// quaternionTFToMsg(quaternion,quat_Msg);//この関数はROSのライブラリ
    //
	// rota_pose.pose.orientation = quat_Msg;

	// cout<<"roll:"<<roll<<",pitch:"<<pitch<<",yaw:"<<yaw<<endl;
	// cout<<"x:"<<a(0, 3)<<",y:"<<a(1, 3)<<",z:"<<a(2, 3)<<endl;
	// cout<<"yaw_"<<yaw_<<endl;

	x_ = a(0, 3);
	y_ = a(1, 3);
	z_ = a(2, 3);

	y_ = cos(buffer_angle_[0])*y_ +sin(buffer_angle_[0])*z_;
	z_ = -sin(buffer_angle_[0])*y_ + cos(buffer_angle_[0])*z_;

	x_ = cos(buffer_angle_[1])*x_ +sin(buffer_angle_[1])*z_;
	z_ = -sin(buffer_angle_[1])*x_ + cos(buffer_angle_[1])*z_;

	x_ = cos(buffer_angle_[2])*x_ +sin(buffer_angle_[2])*y_;
	y_ = -sin(buffer_angle_[2])*x_ + cos(buffer_angle_[2])*y_;


	cout<<x_<<","<<y_<<","<<z_<<endl;

	x_sum += x_;
	y_sum += y_;
	z_sum += z_;

	broadTF_kari(rota_pose,x_sum,y_sum,z_sum,sub_t);	

	std_msgs::Float64 data;

	yaw_judge = yaw;
		
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
Icp::map2transform(vector<double>& dangle ,ros::Time t){
	try{
		
		tflistener_.lookupTransform("/transform", "/map",  
				t, buffer_transform);
		
		tf::Quaternion quatDiff = getQuaternionInverse(buffer_transform.getRotation());
		tf::Matrix3x3(quatDiff).getRPY(dangle[0], dangle[1], dangle[2]);//クォータニオン→オイラー角
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
	
	pub_pc_.header.frame_id  = "/kari_transform";	
	// pub_pc_.header.frame_id  = "/map";	
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

