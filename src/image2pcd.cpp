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
	
	cv::Mat input_img = cv::imread(image_file,0);
	
	int COLS = input_img.cols;	//2048
	int ROWS = input_img.rows;	//126

	pc_cloud->points.clear();
	for(int row=0;row<ROWS;row++){
		for(int col=0;col<COLS;col++){
			float dist_score = input_img.at<uchar>(row, col);

			float dist_score_ = dist_score  * map_limit / MAX_NUM;
			if(dist_score_){
				float phi = (ROWS/2 - row) * (M_PI/2) / (ROWS / 2);
				float theta = (COLS/2 - col) * M_PI / (COLS/2); //pi~-pi

				pcl::PointXYZ pc;
				pc.x = dist_score_ * cos(phi) * cos(theta); 
				pc.y = dist_score_ * cos(phi) * sin(theta); 
				pc.z = dist_score_ * sin(phi);
				pc_cloud->points.push_back(pc);
			}
		}
	}

	sensor_msgs::PointCloud2 pc_pub;
	pcl::toROSMsg(*pc_cloud , pc_pub);           

	pc_pub.header.stamp = ros::Time::now(); 
	pc_pub.header.frame_id = "/map";

	pc_pub_.publish(pc_pub);

}

