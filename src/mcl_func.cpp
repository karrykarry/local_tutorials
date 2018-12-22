#include"mcl.hpp"
#include"mcl_func.hpp"

MCL_F::MCL_F(MCL* mp_MCL):
	odom_sub(n,"/odom",100),imu_sub(n,"/imu/data",100),
	sync(SyncPolicy(10),odom_sub,imu_sub),
	dist(0.0),yaw(0.0),
	start_flag(true)
{
	cout<<"---- function ----"<<endl;
	m_MCL = mp_MCL;
	sync.registerCallback(boost::bind(&MCL_F::syncMsgsCB, this, _1, _2));
}


void
MCL_F::syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){

	double odom_vel,dyaw,dt;
	geometry_msgs::Quaternion odom_quat;

	odom_vel = odom->twist.twist.linear.x;
 	dyaw = imu->angular_velocity.z;
	dt = dt_calc(imu->header.stamp);

	dist += odom_vel * dt; 
	yaw += dyaw * dt;
}

double
MCL_F::dt_calc(ros::Time current_time){
	
	if(start_flag){ 
		last_time = current_time;
		start_flag = false;		
	}

	double last_accurate,current_accurate;
	last_accurate = (double)last_time.nsec*1.0e-9 + last_time.sec;
	current_accurate = (double)current_time.nsec*1.0e-9 + current_time.sec;
	double dt = current_accurate - last_accurate;
	last_time = current_time;

	return dt;
}

void
MCL_F::move_model(vector<Particle>& pf_cloud){

	for(auto itr = pf_cloud.begin(); itr != pf_cloud.end(); ++itr){
		itr->yaw += yaw;

		while(itr->yaw > M_PI) itr->yaw -= 2*M_PI;
		while(itr->yaw < -M_PI) itr->yaw += 2*M_PI;

		itr->x += dist * cos(itr->yaw);// * cos(pitch);
		itr->y += dist * sin(itr->yaw);// * cos(pitch);
	}
	dist = yaw = 0;
}
