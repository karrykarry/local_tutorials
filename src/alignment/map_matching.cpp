/*目標：srv使って
 * 今の位置と位置合わせの位置を与える
*	mapの位合わせをsrvで夜のを作る
 */
#include"map_matching.hpp"


Matcher::Matcher(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	map_limit(20.0),x_now(0.0),y_now(0.0),
	lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	limit_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	map_cloud(new pcl::PointCloud<pcl::PointXYZI>)		//icp後の点群
{
	pc_sub = n.subscribe("/velodyne_obstacles", 100, &Matcher::lidarcallback, this);
	// buffer_pub = n.advertise<sensor_msgs::PointCloud2>("/buffer", 10);
	icp_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_obstacles/ndt", 10);
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/map/vis", 10, true);
	
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
	pc_publisher();
}


void
Matcher::pc_publisher(){

	pcl::toROSMsg(*limit_lidar_cloud , vis_pc);           
	
	vis_pc.header.stamp = ros::Time::now(); //laserのframe_id
	vis_pc.header.frame_id = "/velodyne";

	icp_pub.publish(vis_pc);


}


void
Matcher::map_read(string filename, double size){

	pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1)  
		PCL_ERROR ("事前地図ないよ \n");
	else 
		cout<<"\x1b[31m"<<"読み込んだファイル："<<filename<<"\x1b[m\r"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (size, size, size);
    approximate_voxel_filter.setInputCloud (low_map_cloud);
    approximate_voxel_filter.filter (*map_cloud);


	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time::now(); //laserのframe_id
	vis_map.header.frame_id = "/velodyne";

	map_pub.publish(vis_map);
	sleep(1.0);
}
