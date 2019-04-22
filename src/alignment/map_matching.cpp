/*目標：srv使って
 * 今の位置と位置合わせの位置を与える
*	mapの位合わせをsrvで夜のを作る
 */
#include"map_matching.hpp"


Matcher::Matcher(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	map_limit(20.0),x_now(0.0),y_now(0.0),
	lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	limit_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	map_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//icp後の点群
	local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>)		//icp後の点群
{
	pc_sub = n.subscribe("/velodyne_obstacles", 100, &Matcher::lidarcallback, this);
	// buffer_pub = n.advertise<sensor_msgs::PointCloud2>("/buffer", 10);
	icp_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_obstacles/ndt", 10);
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/map/vis", 10, true);
	lcl_pub = n.advertise<nav_msgs::Odometry>("/ndt_odometry/vis", 10);
	
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);//1.0 change 05/09
	ndt.setMaximumIterations(35);


}


void
Matcher::lidarcallback(const sensor_msgs::PointCloud2::Ptr msg){

	limit_lidar_cloud->points.clear();

	pcl::fromROSMsg(*msg,*lidar_cloud);

	for(pcl::PointXYZI temp_point :lidar_cloud->points){
		
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now) ){
	 
			limit_lidar_cloud->points.push_back(temp_point);
		}
	}
	// pc_publisher();
}



void
Matcher::pc_publisher(){

	pcl::toROSMsg(*local_map_cloud , vis_pc);           
	
	vis_pc.header.stamp = ros::Time::now(); //laserのframe_id
	vis_pc.header.frame_id = "/map";

	icp_pub.publish(vis_pc);

}



void
Matcher::map_read(string filename, double size){

	pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1)  
		PCL_ERROR ("事前地図ないよ \n");
	else 
		cout<<"\x1b[31m"<<"読み込んだファイル："<<filename<<"\x1b[m\r"<<endl;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    
	voxel_filter.setLeafSize (size, size, size);
    voxel_filter.setInputCloud (low_map_cloud);
    voxel_filter.filter (*map_cloud);


	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time::now(); //laserのframe_id
	vis_map.header.frame_id = "/map";

	map_pub.publish(vis_map);
	sleep(1.0);
}

//mapの一部を算出
void 
Matcher::local_map(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    //limit_laserを設定
    float d = 0;
	local_map_cloud->points.clear();

	for(pcl::PointXYZI temp_point :input_cloud->points){
		
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now) ){
	 
			local_map_cloud->points.push_back(temp_point);
		}
	}
}



//位置合わせ
void
Matcher::aligner_ndt(double& roll, double& pitch, double& yaw, double& x, double& y, double& z){
	
	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(roll*(-1), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch*(-1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation (x, y, z);

	Eigen::Matrix4f transform = (rot * init_translation).matrix ();
	
	local_map(map_cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr answer_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
	ndt.setInputTarget(local_map_cloud);	//map
	ndt.setInputSource(limit_lidar_cloud);	//lidar
	ndt.align (*answer_cloud, transform);			//移動後のlidar
	pc_publisher();

 ////answer
 	Eigen::Matrix4f a;
 	a = ndt.getFinalTransformation ();

 	tf::Matrix3x3 mat_l;
	double l_roll, l_pitch, l_yaw;//角度etc
 
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
 			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
 			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

 	mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);
	
	x = a(1, 0);
	y = a(2, 0);
	z = a(3, 0);
	roll = l_roll;
	pitch = l_pitch;
	yaw = l_yaw;
}

void
Matcher::lcl_publisher(double roll, double pitch, double yaw, double x, double y, double z){

	nav_msgs::Odometry lcl_ndt;
		
	lcl_ndt.pose.pose.position.x = x;
	lcl_ndt.pose.pose.position.y = y;
	lcl_ndt.pose.pose.position.z = z;
	
	tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
	geometry_msgs::Quaternion geometry_quat;
	quaternionTFToMsg(quat, geometry_quat);
	lcl_ndt.pose.pose.orientation = geometry_quat;	
	
	lcl_ndt.header.stamp = ros::Time::now();
	lcl_ndt.header.frame_id = "/map";

	lcl_pub.publish(lcl_ndt);

}

