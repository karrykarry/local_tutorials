/* map2image.cpp
 *
 * 2019.06.26
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <map2image.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map2image");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- map2image ----" <<std::endl;
	
	Map2image map2image(n,priv_nh);
	string map_file;
	priv_nh.getParam("map_file",map_file);
	map2image.map_read(map_file);

	ros::Rate loop(1);
	while(ros::ok()){
		map2image.pcd2image();

		loop.sleep();
		ros::spinOnce();
	}
    return 0;
}

       


