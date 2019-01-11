#include"translation_matching.hpp"


lclICP::lclICP(ros::NodeHandle& n) :
{
	pc_sub = n.subscribe("/velodyne_points", 2, &lclICP::pcCallback, this);
	regist_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/icp/registration", 2);
	pc_debug = n.advertise<sensor_msgs::PointCloud2>("/debug/pc_tgt", 1);
	pc_debug2 = n.advertise<sensor_msgs::PointCloud2>("/debug/pc_src", 1);
	state = Matrix4f::Identity();
	pose_tmp.header.frame_id = "/base_link";
	pose_tmp.pose.position.x = pose_tmp.pose.position.y = pose_tmp.pose.position.z
	= pose_tmp.pose.orientation.x = pose_tmp.pose.orientation.y = pose_tmp.pose.orientation.z = 0.0f;
	pose_tmp.pose.orientation.w = 1.0f;

	cout << "src_flag: " << src_flag << endl
		 << "tgt_flag: " << tgt_flag << endl
		 << "dist_flag: " << dist_flag << endl
		 << "first_odom: " << first_odom << endl;
	cloud_num=0;
}


Eigen::Matrix4f 
lclICP::registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	icp.setInputSource(cloud_src_);
	icp.setInputTarget(cloud_tgt_);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	return icp.getFinalTransformation();
}


void 
lclICP::pcCallback(const sensor_msgs::PointCloud2::Ptr msg){
//cout << "	pc Subscribed. " << endl;
	sensor_msgs::PointCloud pc_tmp;
	if(!tgt_flag){
		tgt_t = msg->header.stamp;
		/*------ PointCloud2 -> PointCloud ------*/
		sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_tmp);
		pc_tmp.header.frame_id = "/velodyne";
		pc_tmp.header.stamp = msg->header.stamp;
		/*------ /velodyne -> /odom ------*/
		tgt_flag = getOdomPointCloud(1, pc_tmp);
		getOdomPose(pose_tgt, tgt_t);
	}else{
		if(dist_flag){
			src_t = msg->header.stamp;
			/*------ PointCloud2 -> PointCloud ------*/
			sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_tmp);
			pc_tmp.header.frame_id = "/velodyne";
			pc_tmp.header.stamp = msg->header.stamp;
			/*------ /velodyne -> /odom ------*/
			src_flag = getOdomPointCloud(2, pc_tmp);
			getOdomPose(pose_src, src_t);
			dist_flag = false;
		}
	}

	if(tgt_flag && src_flag){
		/*------ for debug ------*/
		pc_debug_tgt.header.frame_id = pc_debug_src.header.frame_id = "/odom";
		pc_debug_tgt.header.stamp = pc_debug_src.header.stamp = msg->header.stamp;
		pc_debug.publish(pc_debug_tgt);
		pc_debug2.publish(pc_debug_src);


		pc_debug_tgt = pc_debug_src;

	/*	if(norm(pose_src.pose.position.x-pose_tgt.pose.position.x, pose_src.pose.position.y-pose_tgt.pose.position.y, pose_src.pose.position.z-pose_tgt.pose.position.z) > 20){
			tgt_flag = src_flag = false;
			cloud_tgt->points.clear();
			cloud_src->points.clear();
			cout << "Not ICP." << endl;
		}else{*/
			/*------ ICPかけるよ ------*/
			registrationICP();
			/*------ 次のRegistrationに向けてsrcの情報をtgtにコピー&tgtの情報削除 ------*/
			src_flag = false;
			cloud_tgt->points.clear();
			pcl::copyPointCloud(*cloud_src, *cloud_tgt);
			cloud_src->points.clear();
		}
//	}
}

void
lclICP::registrationICP(void){
	/*------ Registration by ICP ------*/
	gettimeofday(&start, NULL);
	Matrix4f F = registration_icp(cloud_tgt, cloud_src);
//	Matrix4f F = registration_ndt(cloud_tgt, cloud_src);
	gettimeofday(&finish, NULL);
	cout << "registration Matrix : " << endl << F << endl;
	calc_time();

		// char pcd_file[3000];
        //
		// sprintf(pcd_file,"/home/amsl/3Dmap/d_kan/clouds/cloud_%d.pcd",cloud_num);
        //
		// cloud_num++;
        //
		// pcl::io::savePCDFileBinary(pcd_file, *cloud_src);



	/*------ Publish ------*/
	geometry_msgs::PoseWithCovarianceStamped regist;
	int cnt = 0;
	for(size_t i=0; i<4; i++){
		for(size_t j=0; j<4; j++){
			regist.pose.covariance[cnt] = F(i,j);
			cnt++;
		}
	}
	regist.header.stamp = ros::Time::now();
	regist_pub.publish(regist);
	cout << "	registration Matrix publish!" << endl;
}

void
lclICP::odomCallback(const nav_msgs::Odometry::Ptr msg){
//cout << "	odom Subscribed. " << endl;
	static double dist;
//cout << "dist : " << dist << endl;
	if(!first_odom){
		first_odom = true;
	}else{
		/*------ 移動距離算出 ------*/
		dist += sqrt( pow((msg->pose.pose.position.x-last_pose.position.x),2.0) + pow((msg->pose.pose.position.y-last_pose.position.y),2.0) );
		/*------ 移動距離が3m以上ならICPかける ------*/
//		if(dist >= 3 && (!tgt_flag)){
		if(dist >= 1){
cout << "1mだよ〜" << endl;
			dist_flag = true;
			dist = 0.0;
		}
	}
	last_pose = msg->pose.pose;
}

bool
lclICP::getOdomPointCloud(int MODE, sensor_msgs::PointCloud pc_tmp){
	sensor_msgs::PointCloud pc_odom;
	sensor_msgs::PointCloud2 pc2_odom;

	try{
		if(MODE == 1){	// tgtの点群取得
			tflistener_.waitForTransform("/odom", "/base_link", pc_tmp.header.stamp, ros::Duration(1.0));
			/*------ /velodyne -> /odom ------*/
			tflistener_.transformPointCloud("/odom", pc_tmp.header.stamp, pc_tmp, "/velodyne", pc_odom);
			/*------ PointCloud -> PointCloud2 ------*/
			sensor_msgs::convertPointCloudToPointCloud2(pc_odom, pc2_odom);
			sensor_msgs::convertPointCloudToPointCloud2(pc_odom, pc_debug_tgt);
			/*------ PointCloud2 -> PCL ------*/
			pcl::fromROSMsg(pc2_odom, *cloud_tgt);
		}else if(MODE == 2){	// srcの点群取得
			tflistener_.waitForTransform("/odom", "/base_link", pc_tmp.header.stamp, ros::Duration(1.0));
			tflistener_.transformPointCloud("/odom", pc_tmp.header.stamp, pc_tmp, "/velodyne", pc_odom);
			sensor_msgs::convertPointCloudToPointCloud2(pc_odom, pc2_odom);
			sensor_msgs::convertPointCloudToPointCloud2(pc_odom, pc_debug_src);
			pcl::fromROSMsg(pc2_odom, *cloud_src);
		}else{
			ROS_ERROR("Not number is input.");
			return false;
		}
	}catch(tf::TransformException ex){
		ROS_ERROR("Failed to compute PointCloud pose.\n%s\n", ex.what());
		return false;
	}

	return true;
}

void
lclICP::getOdomPose(geometry_msgs::PoseStamped &pose, ros::Time t){
	pose_tmp.header.stamp = t;
	try{
		tflistener_.waitForTransform("/odom", "/base_link", t, ros::Duration(1.0));
		tflistener_.transformPose("/odom", t, pose_tmp, "/base_link", pose);
	}catch(tf::TransformException ex){
		ROS_ERROR("Failed to compute odom pose.\n%s\n", ex.what());
	}
}

void
lclICP::calc_time(void){
	float dt = (finish.tv_sec - start.tv_sec) + (finish.tv_usec - start.tv_usec)*1e-6;
	cout << "Calcuration time is : " << dt << "[sec]" << endl;
}

void
lclICP::tf_broad(Matrix4f state, ros::Time t){
	tf::Transform tf;
	// Vector4f quat = mat2quat(state);
	// tf.setOrigin(tf::Vector3(state(0,3), state(1,3), state(2,3)));
	// tf::Quaternion q(quat(1), quat(2), quat(3), quat(0));
	// tf.setRotation(q);
	// tfbroadcaster_.sendTransform(tf::StampedTransform(tf, t, "/map", "/odom"));
}

