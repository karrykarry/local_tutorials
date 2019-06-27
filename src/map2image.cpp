 /*
 */
#include"map2image.hpp"


Map2image::Map2image(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	// width(512), height(64), map_limit(20),
	local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>),
	map_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/map", 10, true);
	local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/local_map", 10);
	image_pub = n.advertise<sensor_msgs::Image>("/vis/map2image", 10);

	private_nh_.getParam("width_pixel",width);
	private_nh_.getParam("height_pixel",height);
	private_nh_.getParam("map_limit",map_limit);
	private_nh_.getParam("pre_path",pre_path);
}

void
Map2image::map_read(string filename){
	double voxel_size = 0.3;

	pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1)  
		PCL_ERROR ("$B;vA0CO?^$J$$$h(B \n");
	else 
		cout<<"\x1b[31m"<<"file name;"<<filename<<"\x1b[m\r"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
	approximate_voxel_filter.setInputCloud (low_map_cloud);
	approximate_voxel_filter.filter (*map_cloud);

	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*low_map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time::now(); 
	vis_map.header.frame_id = "/map";

	map_pub.publish(vis_map);
	sleep(1.0);
	cout<<"\x1b[31m"<<"map read finish"<<filename<<"\x1b[m\r"<<endl;
}

float
Map2image::distance(float x, float y, float z){   
	return sqrt(x*x+y*y+z*z);
}


void
Map2image::visible_map(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv::Mat &based_image){
	
	float dist = 0;
	local_map_cloud->points.clear();
	for(auto point : cloud->points){
		dist = distance(point.x, point.y, point.z);
		if( 0 < dist && dist <= map_limit){
// theta = atan2 (y, x)
			float col = atan2 (point.y, point.x) * 180.0 / M_PI;	//yaw
			int col_ = (int) (col * (-1) * width / 360.0); 			//image$BMQ(B
// phi = acos(z / dist)
			float row = acos(point.z / dist) * 180.0 / M_PI;		//pitch
			int row_ = (int) (row * height / 180.0);

			double dist_score = dist * MAX_NUM / map_limit;
			
			if( based_image.at<uchar>(row_, col_) != 0)
	 			based_image.at<uchar>(row_, col_) = dist_score; 
			else if(based_image.at<uchar>(row_, col_) < dist_score )
	 			based_image.at<uchar>(row_, col_) = dist_score; 

			local_map_cloud->points.push_back(point);
		}
	}
	cout << local_map_cloud->points.size() << endl;
	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*local_map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time::now(); 
	vis_map.header.frame_id = "/map";

	local_map_pub.publish(vis_map);
}


void
Map2image::pcd2image(){
	static int i = 0;
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 3 ) << i++;


	cv::Mat based_image = cv::Mat::zeros(height, width, CV_8UC1);
	
	visible_map(map_cloud, based_image);
	 	
	cv::imshow("MAP2DEPTH", based_image);
	cv::imwrite(pre_path + oss.str() + ".png", based_image);
	cv::waitKey(2);
}

