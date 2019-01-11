#include"mcl.hpp"
#include"mcl_func.hpp"

MCL::MCL(ros::NodeHandle n,ros::NodeHandle priv_nh):
	pf_num(1000),start_fg(false)
{
	server.setCallback(boost::bind(&MCL::paramcallback, this, _1, _2));
	pf_pub = n.advertise<geometry_msgs::PoseArray>("/pf_cloud",100);
	lcl_pub = n.advertise<nav_msgs::Odometry>("/lcl_pf",100);

	pf_array.header.frame_id = "/map";
	lcl_pf.header.frame_id = "/map";
	lcl_pf.child_frame_id = "/matching_base_link";

	m_MCL_F = new MCL_F(this);
	init_set();
}

//dynamic_reconfigure/*{{{*/
void 
MCL::paramcallback(local_tutorials::mcl_paramConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %d", 
			config.particle_num, config.const_num);

	pf_num = config.particle_num;
}/*}}}*/

void
MCL::init_set(){
	Particle pf;

	for(int i=0;i<pf_num;i++){
	std::random_device rnd;
	mt19937 mt(rnd());
	uniform_real_distribution<double> score(-5.0,5.0);
	uniform_real_distribution<double> yaw_score(-1.0,1.0);

	pf.x = score(mt);
	pf.y = score(mt);
	pf.yaw = yaw_score(mt) / 2.0;

	pf.weight = 1.0/pf_num;
	pf_cloud.push_back(pf);
	}
}

int cnt=0;
void
MCL::pub(){
	geometry_msgs::Pose pf_geo;

	int pf_size = (int)pf_cloud.size();
	pf_array.poses.clear();
	m_MCL_F->reset();
	pf_array.poses.resize(pf_size);
	
	double lcl_x,lcl_y,lcl_yaw;
	double weight_sum,ess;
	lcl_x = lcl_y = lcl_yaw =  0.0;
	weight_sum = ess = 0.0;

	m_MCL_F->move_model(pf_cloud);
	 // #pragma omp parallel for
	for(int i=0;i<pf_size;i++){
		
		pf_cloud[i].weight = m_MCL_F->measurement_model(pf_cloud[i].x,pf_cloud[i].y,pf_cloud[i].yaw);
		
		pf_geo = pf2geo(pf_cloud[i]);

		pf_array.poses[i] = pf_geo;

		//lcl
		lcl_x += pf_geo.position.x*pf_cloud[i].weight;
		lcl_y += pf_geo.position.y*pf_cloud[i].weight;
		lcl_yaw += pf_cloud[i].yaw*pf_cloud[i].weight;
		weight_sum += pf_cloud[i].weight;
		ess += pow(pf_cloud[i].weight,2); 
	
	}
	cout << "ess:" << ess << endl;

	pf_array.header.stamp = ros::Time::now();
	pf_pub.publish(pf_array);


	if(cnt>30 && cnt%5==0){	
		m_MCL_F->resample(pf_cloud,new_pf_cloud);

		pf_cloud = new_pf_cloud; 

		new_pf_cloud.clear();
	}
	cnt++;

	lcl_x /= weight_sum;
	lcl_y /= weight_sum;
	lcl_yaw /= weight_sum;

	while(lcl_yaw > M_PI) lcl_yaw -= 2*M_PI;
	while(lcl_yaw < -M_PI) lcl_yaw += 2*M_PI;

	lcl_pub_(lcl_x,lcl_y,lcl_yaw);
	


}


//pf2geo/*{{{*/
geometry_msgs::Pose
MCL::pf2geo(Particle pf){
	geometry_msgs::Pose pf_geo;
	
	pf_geo.position.x = pf.x;
	pf_geo.position.y = pf.y;
	pf_geo.position.z = 0.0;
	pf_geo.orientation.x = 0.0;
	pf_geo.orientation.y = 0.0;
	pf_geo.orientation.z = sin(pf.yaw*0.5);
	pf_geo.orientation.w = cos(pf.yaw*0.5);

	return pf_geo;
}/*}}}*/

//lcl tf pub/*{{{*/
void
MCL::lcl_pub_(double x, double y,double yaw){

	lcl_pf.pose.pose.position.x = x;
	lcl_pf.pose.pose.position.y = y;
	lcl_pf.pose.pose.position.z = 0.0;
	lcl_pf.pose.pose.orientation.x = 0.0;
	lcl_pf.pose.pose.orientation.y = 0.0;
	lcl_pf.pose.pose.orientation.z = sin(yaw*0.5);
	lcl_pf.pose.pose.orientation.w = cos(yaw*0.5);
	
	lcl_pf.header.stamp = ros::Time::now();

	lcl_pub.publish(lcl_pf);

	transform.setOrigin( tf::Vector3( x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, yaw);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, lcl_pf.header.stamp, "/map", "/matching_base_link"));

}/*}}}*/


