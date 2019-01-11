/* translation.cpp
 *
 * 2018.12.21
 *
 * author : R.Kusakari
 *
*/ 
#include <ros/ros.h>
#include "translation_matching.hpp"

using namespace std;

    
int main(int argc, char* argv[]){
	ros::init(argc, argv, "translation_matching");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	Icp icp_(n,priv_nh);
	cout<<"----- icp start --------"<<endl;

	ros::spin();

	return 0; 
}
