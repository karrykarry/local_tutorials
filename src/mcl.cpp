#include"mcl.hpp"

MCL::MCL(ros::NodeHandle n,ros::NodeHandle priv_nh)
{
	server.setCallback(boost::bind(&MCL::paramcallback, this, _1, _2));
	pf_pub = n.advertise<geometry_msgs::PoseArray>("/pf_cloud",100);

	pf_array.header.frame_id = "/map";
}

void 
MCL::paramcallback(local_tutorials::mcl_paramConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %d", 
			config.particle_num, config.const_num);

	pf_num = config.particle_num;
}


Particle
MCL::pf_state(){
	Particle pf;

	std::random_device rnd;
	mt19937 mt(rnd());
	uniform_real_distribution<double> score(-5.0,5.0);
	
	pf.x = score(mt);
	pf.y = score(mt);
	pf.yaw = 0.0;
	pf.weight = 0.0;

	return pf;
}

geometry_msgs::Pose
MCL::pf2geo(Particle pf){
	geometry_msgs::Pose pf_geo;
	
	pf_geo.position.x = pf.x;
	pf_geo.position.y = pf.y;
	pf_geo.position.z = 0.0;
	pf_geo.orientation.x = 0.0;
	pf_geo.orientation.y = 0.0;
	pf_geo.orientation.z = 0.0;
	pf_geo.orientation.w = 1.0;

	return pf_geo;
}

void
MCL::pub(){
	Particle pf;
	geometry_msgs::Pose pf_geo;

	pf_array.poses.clear();
	for(int i=0;i<pf_num;i++){
		pf = pf_state();
		pf_geo = pf2geo(pf);

		pf_array.poses.push_back(pf_geo);
	}
	pf_array.header.stamp = ros::Time::now();
	pf_pub.publish(pf_array);
	
}
