/* map_matcher_main.cpp
 *
 * 2019.04.24
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include <map_matching_tf.hpp>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_matching_tf");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	std::cout<<"---- main ----" <<std::endl;
	
	Map_tf map_tf(n,priv_nh);
	
	string from_id,to_id;

	priv_nh.getParam("from_id",from_id);
	priv_nh.getParam("to_id",to_id);

	ros::Rate loop(10);
	while(ros::ok()){
		map_tf.listen_tf(from_id,to_id);
		
		loop.sleep();
		ros::spinOnce();
	}

    return 0;
}

       



