/* particle.cpp
 *
 * 2018.12.21
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <map_matching.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_alignment");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- map matching ----" <<std::endl;
	
	Matcher map_matcher(n,priv_nh);
	string map_file;
	double map_voxel_size;
	priv_nh.getParam("map_file",map_file);
	priv_nh.getParam("map_voxel_size",map_voxel_size);
	map_matcher.map_read(map_file,map_voxel_size);
	
	ros::Rate loop(10);
	while(ros::ok()){

		loop.sleep();
		ros::spinOnce();
	}

    return 0;
}

       


