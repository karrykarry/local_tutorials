 /*
 */
#include"image2pcd.hpp"


Image2pcd::Image2pcd(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	pc_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	pc_pub_ = n.advertise<sensor_msgs::PointCloud2>("/create_pc", 10);

	private_nh_.getParam("map_limit",map_limit);
}




void
Image2pcd::create_pcd(string image_file){
	
	cv::Mat input_img = cv::imread(image_file,1);
	
	int COLS = input_img.cols;
	int ROWS = input_img.rows;
	
	pc_cloud->points.clear();
	for(int row=0;row<ROWS;row++){
		for(int col=0;col<COLS;col++){
			float dist_score = input_img.at<uchar>(row, col) * map_limit / MAX_NUM;
			// float phi = row * M_PI / ROWS;
			float phi = (ROWS/2 - row) * M_PI / ROWS;
			float theta = (COLS/2 - col) * 2*M_PI / COLS; //pi~-pi
			
			pcl::PointXYZ pc;
			// pc.x = dist_score * sin(theta) * cos(phi); 
			// pc.y = dist_score * sin(theta) * sin(phi); 
			// pc.z = dist_score * cos(theta);
			pc.x = dist_score * cos(phi) * cos(theta); 
			pc.y = dist_score * cos(phi) * sin(theta); 
			pc.z = dist_score * sin(phi);
			if(dist_score) pc_cloud->points.push_back(pc);
		}
	}
	// cout << pc_cloud->points.size() << endl;
	cout << COLS << "," << ROWS << endl;

	sensor_msgs::PointCloud2 pc_pub;
	pcl::toROSMsg(*pc_cloud , pc_pub);           

	pc_pub.header.stamp = ros::Time::now(); 
	pc_pub.header.frame_id = "/map";

	pc_pub_.publish(pc_pub);

}

