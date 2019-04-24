/* map_matcher.cpp
 *
 * 2019.04.24
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

	std::cout<<"----Node : map matching ----" <<std::endl;
	
	Matcher map_matcher(n,priv_nh);
	string map_file;
	priv_nh.getParam("map_file",map_file);
	map_matcher.map_read(map_file);
	
	ros::spin();

    return 0;
}

       


