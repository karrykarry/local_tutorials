/* occupancy_voxel_grid.cpp
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <ovg.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ovg");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- ovg_start ----" <<std::endl;


	OVG ovg(n,priv_nh);

	ros::spin();

    return 0;
}

       


