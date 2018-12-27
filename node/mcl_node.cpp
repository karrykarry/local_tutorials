/* particle.cpp
 *
 * 2018.12.21
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <mcl.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mcl");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- mcl ----" <<std::endl;
	
	MCL mcl(n,priv_nh);
	
	ros::Rate loop(5);
	while(ros::ok()){
		if(mcl.start_fg)mcl.pub();
		else std::cout<<"not start_fg"<<std::endl;


		loop.sleep();
		ros::spinOnce();
	}

    return 0;
}

       

