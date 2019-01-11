#include"mcl.hpp"
#include"mcl_func.hpp"

MCL_F::MCL_F(MCL* mp_MCL):
	lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),
	odom_sub(n,"/odom",100),imu_sub(n,"/imu/data",100),
	sync(SyncPolicy(10),odom_sub,imu_sub),
	dist(0.0),yaw(0.0),
	start_flag(true),
	cell_filter(1),local_width(20),local_height(20),R(0.1),min_x(0.0),min_y(0.0),
	weight_sum(0.0)
{
	cout<<"---- function ----"<<endl;
	m_MCL = mp_MCL;
	sync.registerCallback(boost::bind(&MCL_F::syncMsgsCB, this, _1, _2));
	grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/lidar_grid", 10);
	map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map", 1, &MCL_F::gridmapCallback, this);
	lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &MCL_F::lidarCallback, this);
}

void
MCL_F::syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){
/*{{{*/
	double odom_vel,dyaw,dt;
	geometry_msgs::Quaternion odom_quat;

	odom_vel = odom->twist.twist.linear.x;
 	dyaw = imu->angular_velocity.z;
	dt = dt_calc(imu->header.stamp);

	dist += odom_vel * dt; 
	yaw += dyaw * dt;
	
	m_MCL->start_fg = true;
	/*}}}*/

}

void
MCL_F::gridmapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg){
	map_grid = *map_msg;
	R = map_msg->info.resolution;

	local_map.data.resize(int(local_width/R) * int(local_height/R));
	local_map.info.width = int(local_width/R);
	local_map.info.height = int(local_height/R);
	local_map.info.resolution = R;
	local_map.info.origin.position.x = (min_x - local_width)/2.0; 
	local_map.info.origin.position.y = (min_y - local_height)/2.0;


}

void 
MCL_F::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::fromROSMsg (*input, *lidar_cloud);

}


double
MCL_F::dt_calc(ros::Time current_time){
	/*{{{*/
	if(start_flag){ 
		last_time = current_time;
		start_flag = false;		
	}

	double last_accurate,current_accurate;
	last_accurate = (double)last_time.nsec*1.0e-9 + last_time.sec;
	current_accurate = (double)current_time.nsec*1.0e-9 + current_time.sec;
	double dt = current_accurate - last_accurate;
	last_time = current_time;

	return dt;/*}}}*/
}


void
MCL_F::move_model(vector<Particle>& pf_clouds){
/*{{{*/
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	std::normal_distribution<> nd_(1.0,0.2);	//平均1.0、標準偏差0.2で分布させる

	for(auto& pf_cloud : pf_clouds){ 

		pf_cloud.yaw += yaw * nd_(engine);

		while(pf_cloud.yaw > M_PI) pf_cloud.yaw -= 2*M_PI;
		while(pf_cloud.yaw < -M_PI) pf_cloud.yaw += 2*M_PI;

		pf_cloud.x += dist * nd_(engine) * cos(pf_cloud.yaw);// * cos(pitch);
		pf_cloud.y += dist * nd_(engine) * sin(pf_cloud.yaw);// * cos(pitch);
	}

	dist = yaw = 0;/*}}}*/
}



void 
MCL_F::create_obstacle_map(pcl::PointCloud<pcl::PointXYZI>::Ptr clouds, nav_msgs::OccupancyGrid *map){
	
	vector<int>	count((long(local_width/R) * long(local_height/R)), 0);

	for(auto cloud : clouds->points){

		int x = int((cloud.x - map->info.origin.position.x) / R);
		int y = int((cloud.y - map->info.origin.position.y) / R);
		if((0 <= x && x < local_width/R) && (0 <= y && y < local_height/R)){
			long num = x + y * map->info.width;
			count[num] += 1; 
			if(count[num] > cell_filter){
				map->data[num] = 100;	
			}
		}
	}


}



double
MCL_F::measurement_model(double x,double y,double yaw){
	

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	transform.rotate (Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud (*lidar_cloud, *transformed_cloud, transform);

	vector<bool> init_flag((long(local_width/R) * long(local_height/R)), false);
	int same_c;
	double obstacle_c;

	same_c = 0;
	obstacle_c = 0.0;
	for(size_t i = 0; i < transformed_cloud->points.size(); i++){
		int local_x = int((transformed_cloud->points[i].x - local_map.info.origin.position.x) / R);
		int local_y = int((transformed_cloud->points[i].y - local_map.info.origin.position.y) / R);
		if((0 <= local_x && local_x < local_width/R) &&
				(0 <= local_y && local_y < local_height/R)){
			long num = local_x +local_y * local_map.info.width;
			if(!init_flag[num]){
	
				int global_x = int((transformed_cloud->points[i].x + x - map_grid.info.origin.position.x) / R);
				int global_y = int((transformed_cloud->points[i].y + y - map_grid.info.origin.position.y) / R);

				if((0 <= global_x  && global_x < map_grid.info.width/R) &&
						(0 <= global_y && global_y < map_grid.info.height/R)){
					long global_num = global_x +global_y * map_grid.info.width;
					if(map_grid.data[global_num]==100){
						same_c++;
					}
				}
				obstacle_c++;
				init_flag[num] = true;
			}
		}
	}

	weight_sum += (double)same_c/obstacle_c;
	weight_array.push_back(weight_sum);


	return (double)same_c/obstacle_c;

}


//リサンプリング
void 
MCL_F::resample(vector<Particle> pf_cloud,vector<Particle>& new_pf_cloud){

	double ep;

    size_t pt_size = pf_cloud.size();
	
	Particle new_pf;

	std::random_device rnd;
	mt19937 mt(rnd());
	uniform_real_distribution<double> ep_(0,weight_sum);
	ep = ep_(mt);

    for(size_t i=0;i<pt_size;i++){
        ep = ep + ((double)weight_sum/pt_size);

        if(ep>weight_sum){
            ep = ep - weight_sum;
        }

        new_pf.weight = 1.0 / pt_size;

        for(size_t j=0;j<pt_size;j++){//ルーレット
            // if(ep>w_sum[j]){//静的
            if(ep>weight_array.at(j)){//動的               
                continue;
            }

            else{

				new_pf.x = pf_cloud[j].x;
				new_pf.y = pf_cloud[j].y;
				new_pf.yaw = pf_cloud[j].yaw;

                new_pf_cloud.push_back(new_pf);

                break;

            }
        }
    }
}

void
MCL_F::reset(){
		weight_array.clear();
		weight_sum = 0.0;
}

void
MCL_F::say(){
		cout<<"weight_array"<<weight_array.size()<<endl;
}
