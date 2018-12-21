#include"mcl.hpp"

MCL::MCL(ros::NodeHandle n,ros::NodeHandle priv_nh)
{
}

Particle
MCL::pf_state(){
	Particle pf;

	std::random_device rnd;
	mt19937 mt(rnd());
	uniform_real_distribution<double> score(0.0,1.0);
	
	pf.x = score(mt);
	pf.y = score(mt);
	pf.yaw = 0.0;
	pf.weight = 0.0;

	return pf;
}

void
MCL::pub(){
	Particle pf;
	pf = pf_state();

	cout<<pf.x<<","<<pf.y<<endl;
	
}
