/* map2image.cpp
 *
 * 2019.06.26
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <image2pcd.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image2pcd");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- image2pcd ----" <<std::endl;
	
	Image2pcd image2pcd(n,priv_nh);
	string image_file;
	priv_nh.getParam("image_file",image_file);

	ros::Rate loop(10);
	while(ros::ok()){
		image2pcd.create_pcd(image_file);

		loop.sleep();
		ros::spinOnce();
	}
    return 0;
}

       


