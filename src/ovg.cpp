#include"ovg.hpp"


OVG::OVG(ros::NodeHandle n,ros::NodeHandle private_nh_):
	voxel_CoG (new pcl::PointCloud<pcl::PointXYZINormal>)
	,voxel_static (new pcl::PointCloud<pcl::PointXYZINormal>)
	,voxel_free (new pcl::PointCloud<pcl::PointXYZINormal>)
	,voxel_probability (new pcl::PointCloud<pcl::PointXYZINormal>)
	,voxel_normal (new pcl::PointCloud<pcl::PointXYZINormal>)
	,voxel_occlusion (new pcl::PointCloud<pcl::PointXYZINormal>)
	,cloud_idx (new pcl::PointCloud<pcl::PointXYZINormal>)
	,cloud_static (new pcl::PointCloud<pcl::PointXYZINormal>)

{
	pc_sub = n.subscribe("/velodyne_points", 1, &OVG::pcCallback, this);
	
	static_voxel_pub = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/static", 2);
	free_voxel_pub = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/free", 2);
	prob_voxel_pub = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/probability", 2);
	occlusion_voxel_pub_ = n.advertise<sensor_msgs::PointCloud2>("/voxelGrid/occlusion", 2);;
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/probability/raw", 2);
	static_points_pub = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/static", 2);
	
	pc_debug_1 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug1", 2);
	pc_debug_2 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug2", 2);
	pc_debug_3 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug3", 2);
	pc_debug_4 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug4", 2);
	pc_debug_5 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug5", 2);
	pc_debug_6 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud/debug6", 2);
	
	pcpose_pub = n.advertise<original_msgs::pointsWithPose>("/PointCloudWithPose/probability", 2);
	
	private_nh_.getParam("max_x_range",max_x_range);
	private_nh_.getParam("max_y_range",max_y_range);
	private_nh_.getParam("max_z_range",max_z_range);
	private_nh_.getParam("voxel_size",voxelSize);
	private_nh_.getParam("SKIP",skip);
	private_nh_.getParam("STEP",step);
	private_nh_.getParam("probability_threshold",prob_threshold);
	private_nh_.getParam("save_raw_flag",save_raw_flag);
	private_nh_.getParam("debug_pub_flag",debug_pub_flag);

	pc_vec = vector<sensor_msgs::PointCloud>(step);	//データ格納変数
	
	setVoxelGrid(voxel_CoG);
	is_shot_pc = vector<vector<int> >(voxel_CoG->points.size(), vector<int>(step, 0));
	init_pose.header.frame_id = "/base_link";
	init_pose.pose.position.x = 0.0; init_pose.pose.position.y = 0.0; init_pose.pose.position.z = 0.0;
	init_pose.pose.orientation.x = 0.0; init_pose.pose.orientation.y = 0.0; init_pose.pose.orientation.z = 0.0; 
	init_pose.pose.orientation.w = 1.0;


	printf("MAX RANGE :(%.0lf[m] x %.0lf[m] x %.0lf[m])\n", max_x_range, max_y_range, max_z_range);
	printf("voxel size : %.2lf[m]\n", voxelSize);
	cout << "SKIP : " << skip << endl << "STEP : " << step << endl;
	cout << "PROB_THESHOLD : " << prob_threshold << endl;
	printf("save raw flag : %s\n", save_raw_flag ? "true" : "false");
	printf("debug pub flag : %s\n", debug_pub_flag ? "true" : "false");
	cout << "All systems are ready." << endl;
}

void
OVG::setVoxelGrid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud){
	int x = max_x_range / voxelSize;
	int y = max_y_range / voxelSize;
	int z = max_z_range / voxelSize;
	int count = 0;
	double init_x = (-1 * max_x_range + voxelSize)/2;
	double init_y = (-1 * max_y_range + voxelSize)/2;
	double init_z = (-1 * max_z_range + voxelSize)/2;

	cloud->points.resize(x*y*z);
	for(int k=0; k<z; k++){
		for(int j=0; j<y; j++){
			for(int i=0; i<x; i++){
				cloud->points[count].x = init_x + voxelSize*i;
				cloud->points[count].y = init_y + voxelSize*j;
				cloud->points[count].z = init_z + voxelSize*k;
				cloud->points[count].intensity = 0.0;
				cloud->points[count].normal_x = 0.0;
				cloud->points[count].normal_y = 0.0;
				cloud->points[count].normal_z = 0.0;
				count++;
			}
		}
	}
	cout << "Num of voxel is " << cloud->points.size() << "(" << x << "cell x " << y << "cell x " << z << "cell)" << endl;
}



void
OVG::pcCallback(const sensor_msgs::PointCloud2::Ptr msg){
	static int seq;	// velodyneのデータを何回受け取ったかカウントする変数
	static int cnt;	// sensor_msgsのvectorの何番目にデータを格納するかの変数
//	cout << "sensor_msgs::PointCloud2 is subscribed. " << seq << endl;
	if(seq%skip == 0){
//		cout << "	Transform /velodyne -> /odom" << endl;
		sensor_msgs::PointCloud pc_tmp_local;	// PC2->PC へ一時変換用変数
		sensor_msgs::PointCloud pc_tmp_global;
		sub_time_ = msg->header.stamp;
		pc_tmp_local.header.stamp = msg->header.stamp;
		pc_tmp_local.header.frame_id = "/velodyne";
		/*------ PointCloud2 -> PointCloud ------*/
		sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_tmp_local);
		/*------ /velodyne -> /odom ------*/
		try{
			tflistener.waitForTransform("/odom", "/velodyne", sub_time_, ros::Duration(1.0));
			tflistener.transformPointCloud("/odom", sub_time_, pc_tmp_local, "/velodyne", pc_tmp_global);
			if(cnt > step){
				if(save_raw_flag){
					save_rawData(msg);
				}
//cout << "pcCallback" << endl;
				calc_nowPC_isShot(msg);
//				supplement(1);
				pcHandle();
				is_shot_nowPC.clear(); is_shot_nowPC.shrink_to_fit();
			}
			pc_vec[cnt%step] = pc_tmp_global;
			cnt++;
		}catch(tf::TransformException ex){
			ROS_ERROR("%s\n", ex.what());
		}
	}
	seq++;
	
}

void
OVG::pcHandle(void){
//cout << "pcHandle" << endl;
//	cout << "	Transform /odom -> /velodyne" << endl;
gettimeofday(&start, NULL);
	vector<pcl::PointCloud<pcl::PointXYZINormal> > pcl_vec(step);
	transformOdom2velodyne(pcl_vec, sub_time_);		// pc2_vecに時刻sub_time_におけるvelodyne座標系原点から見た過去6ステップ分の点群情報を格納
	calc_pcPoint(pcl_vec, is_shot_pc);		// 各空間に点群が存在するかをstep数だけ判定. 計算結果はis_shot_pc内にてint型で記述
gettimeofday(&finish, NULL);
dt = (finish.tv_sec - start.tv_sec) + (finish.tv_usec - start.tv_usec)*1e-6;
cout << "Calcuration time is : " << dt << "[sec]" << endl;
	int voxel_size = (int)voxel_CoG->points.size();
	vector<double> prob_voxel(voxel_size, 0.0);
	for(int i=0; i<voxel_size; i++){
		// 各グリッドセルの確率を計算
		int count = 0;
		for(int j=0; j<step; j++){
			if(is_shot_pc[i][j] != -1){
				prob_voxel[i] += (double)is_shot_pc[i][j];
				count++;
			}
		}
		if(count!=0){
//			cout << "voxel[" << i << "] : " << prob_voxel[i] << ", count : " << count << ", ";
			prob_voxel[i] /= (double)count;
//			cout << prob_voxel[i] << endl;
		}else if(count==0){
			prob_voxel[i] = 0.0;
		}
		// グリッドセルの確率に応じて各PCにデータ格納
		if(voxel_normal->points[i].intensity != 0.0){
			voxel_normal->points[i].normal_x /= voxel_normal->points[i].intensity;
			voxel_normal->points[i].normal_y /= voxel_normal->points[i].intensity;
			voxel_normal->points[i].normal_z /= voxel_normal->points[i].intensity;
			voxel_normal->points[i].x /= voxel_normal->points[i].intensity;
			voxel_normal->points[i].y /= voxel_normal->points[i].intensity;
			voxel_normal->points[i].z /= voxel_normal->points[i].intensity;
			//voxel_normal->points[i].intensity = prob_threshold;
			voxel_normal->points[i].intensity = prob_voxel[i];
		}
		if( prob_voxel[i] != 0.0 ){
			voxel_probability->points.push_back(voxel_CoG->points[i]);
//			voxel_probability->points.resize(voxel_probability->points.size()+1);
//			voxel_probability->points[voxel_probability->points.size()-1].x = voxel_CoG->points[i].x;
//			voxel_probability->points[voxel_probability->points.size()-1].y = voxel_CoG->points[i].y;
//			voxel_probability->points[voxel_probability->points.size()-1].z = voxel_CoG->points[i].z;
			voxel_probability->points[voxel_probability->points.size()-1].intensity = prob_voxel[i];
		}
		if( prob_voxel[i] >= prob_threshold ){	// 確率が閾値より高い空間
			voxel_static->points.push_back(voxel_CoG->points[i]);
		}
		if(is_shot_nowPC[i] == -1 && prob_voxel[i]>=0.5f){
			voxel_occlusion->points.push_back(voxel_normal->points[i]);
//			int voxSize = voxel_occlusion->points.size()-1;
//			voxel_occlusion->points[voxSize].x = voxel_CoG->points[i].x;
//			voxel_occlusion->points[voxSize].y = voxel_CoG->points[i].y;
//			voxel_occlusion->points[voxSize].z = voxel_CoG->points[i].z;
		}
		if( is_shot_nowPC[i] == 0 ){	// 現在の点群が通過した空間
			voxel_free->points.push_back(voxel_CoG->points[i]);
		}else{
			continue;
		}
	}
	int cloud_size = (int)cloud_idx->points.size();
	for(int i=0; i<cloud_size; i++){
		int idx = (int)cloud_idx->points[i].intensity;		// この時は、.intentityに格納されているのはvoxel_CoGのidx番号(=i番目の点群がintensity番目のグリッドセルに属する)
		cloud_idx->points[i].intensity = prob_voxel[idx];	// ここで、.intensityに確率を格納
		//cloud_idx->points[i].intensity = 1.0;	// ここで、.intensityに確率を格納
		if(cloud_idx->points[i].intensity >= prob_threshold){
			cloud_static->points.push_back(cloud_idx->points[i]);
		}
//		cout << "pointCloud[" << i << "].prob : " << cloud_idx->points[i].intensity << "prob_voxel : " << prob_voxel[idx] << endl;
	}
	//*cloud_idx += *voxel_occlusion;
	/*------ PCL -> ROS ------*/
	sensor_msgs::PointCloud2 voxelGrid_static;	// 静的障害物があるっぽいボクセル
	sensor_msgs::PointCloud2 voxelGrid_free;	// 動的障害物が(ry
	sensor_msgs::PointCloud2 voxelGrid_probability;
	sensor_msgs::PointCloud2 voxelGrid_occlusion;
	sensor_msgs::PointCloud2 pointCloud_probability;
	sensor_msgs::PointCloud2 pointCloud_static;
	original_msgs::pointsWithPose pcWithPose_probability;
	pcl::toROSMsg(*voxel_static, voxelGrid_static);
	pcl::toROSMsg(*voxel_free, voxelGrid_free);						//cout << "\x1b[46mFree cloud size : " << voxel_free->points.size() << "\x1b[0m" << endl;
	pcl::toROSMsg(*voxel_probability, voxelGrid_probability);
	pcl::toROSMsg(*voxel_occlusion, voxelGrid_occlusion);
	pcl::toROSMsg(*cloud_idx, pointCloud_probability);
	pcl::toROSMsg(*cloud_static, pointCloud_static);
	voxelGrid_static.header.frame_id = voxelGrid_free.header.frame_id = "/velodyne";
	voxelGrid_probability.header.frame_id = "/velodyne";
	voxelGrid_occlusion.header.frame_id = "/velodyne";
	pointCloud_probability.header.frame_id = "/velodyne";
	pointCloud_static.header.frame_id = "/velodyne";
	voxelGrid_static.header.stamp = voxelGrid_free.header.stamp = sub_time_;
	voxelGrid_probability.header.stamp = sub_time_;
	voxelGrid_occlusion.header.stamp = sub_time_;
	pointCloud_probability.header.stamp = sub_time_;
	pointCloud_static.header.stamp = sub_time_;
	pcWithPose_probability.points = pointCloud_probability;
	if(getPose(pcWithPose_probability.pose, sub_time_)){
		broad_tf(pcWithPose_probability);
		pcpose_pub.publish(pcWithPose_probability);
	}
	static_voxel_pub.publish(voxelGrid_static);
    free_voxel_pub.publish(voxelGrid_free);
	prob_voxel_pub.publish(voxelGrid_probability);
	occlusion_voxel_pub_.publish(voxelGrid_occlusion);
	pc_pub.publish(pointCloud_probability);
	static_points_pub.publish(pointCloud_static);
//	cout << "published!" << endl;

	voxel_free->points.clear();
	voxel_static->points.clear();
	voxel_probability->points.clear();
	voxel_normal->points.clear();
	voxel_occlusion->points.clear();
	cloud_idx->points.clear();
	cloud_static->points.clear();
}

void
OVG::transformOdom2velodyne(vector<pcl::PointCloud<pcl::PointXYZINormal> >& pcl_vec, ros::Time t){
//cout << "transformOdom2velodyne" << endl;
	vector<sensor_msgs::PointCloud2> pc2_vec(step);
	vector<sensor_msgs::PointCloud> pc_vel_tmp(step);
	int pc_vec_size = pc_vec.size();

	for(int i=0; i<pc_vec_size;){
		pc_vel_tmp[i].header.stamp = t;
		pc_vel_tmp[i].header.frame_id = "/velodyne";
		try{
			tflistener.waitForTransform("/velodyne", "/odom", t, ros::Duration(1.0));
			tflistener.transformPointCloud("/velodyne", t, pc_vec[i], "/odom", pc_vel_tmp[i]);
			sensor_msgs::convertPointCloudToPointCloud2(pc_vel_tmp[i], pc2_vec[i]);
			pcl::fromROSMsg(pc2_vec[i], pcl_vec[i]);
			/*------ For debug ------*/
			pc2_vec[i].header.frame_id = "/velodyne";
			pc2_vec[i].header.stamp = t;
			i++;
		}catch(tf::TransformException ex){
			ROS_ERROR("%s\n", ex.what());
		}
	}
	if(debug_pub_flag){
		pc_debug_1.publish(pc2_vec[1]);
		pc_debug_2.publish(pc2_vec[4]);
		pc_debug_3.publish(pc2_vec[7]);
		pc_debug_4.publish(pc2_vec[10]);
		pc_debug_5.publish(pc2_vec[13]);
		pc_debug_6.publish(pc2_vec[15]);
	}
}


void
OVG::calc_pcPoint(vector<pcl::PointCloud<pcl::PointXYZINormal> > pcl_vec, vector<vector<int> >& is_shot){
//cout << "calc_pcPoint" << endl;
//	cout << "max range : (" << max_x_range << "[m], " << max_y_range << "[m], " << max_z_range << "[m])" << endl;
	is_shot = vector<vector<int> >(voxel_CoG->points.size(), vector<int>(pcl_vec.size(), -1));	// is_shot[ボクセル空間のボクセル数][貯めたスキャン数]
	/*---- i:点群数 j:スキャン数 ----*/
	int voxel_size = (int)voxel_CoG->points.size();
	voxel_normal->points.resize(voxel_size);
	size_t vec_size = pcl_vec.size();
	for(size_t j=0; j<vec_size; j++){
		size_t cloud_size = pcl_vec[j].points.size();
	//	cout << "input data is :" << cloud_size << " data." << endl;
		for(size_t i=0; i<cloud_size; i++){
	//		cout << "i : " << i << endl;
			/*---- 占有空間 ----*/
			if( ( fabs(pcl_vec[j].points[i].x) <= (max_x_range/2))  &&
				( fabs(pcl_vec[j].points[i].y) <= (max_y_range/2))  &&
				( fabs(pcl_vec[j].points[i].z) <= (max_z_range/2))  ){
				int Xidx_occupied = (int)( ((max_x_range/2) + pcl_vec[j].points[i].x) / voxelSize );
				int Yidx_occupied = (int)( ((max_y_range/2) + pcl_vec[j].points[i].y) / voxelSize );
				int Zidx_occupied = (int)( ((max_z_range/2) + pcl_vec[j].points[i].z) / voxelSize );
				int idx_occupied = (int)( Xidx_occupied + (Yidx_occupied*(max_x_range/voxelSize)) + (Zidx_occupied*(max_x_range/voxelSize)*(max_y_range/voxelSize)) );
				if(idx_occupied<voxel_size){
					if(is_shot[idx_occupied][j] != 1){
						is_shot[idx_occupied][j] = 1;
					}
					if(isnan(pcl_vec[j].points[i].normal_x) || isnan(pcl_vec[j].points[i].normal_y) || isnan(pcl_vec[j].points[i].normal_z)){
						cout << "\x1b[41mNanがいるぞ！\x1b[0m" << endl;
					}
					voxel_normal->points[idx_occupied].x += pcl_vec[j].points[i].x;
					voxel_normal->points[idx_occupied].y += pcl_vec[j].points[i].y;
					voxel_normal->points[idx_occupied].z += pcl_vec[j].points[i].z;
					voxel_normal->points[idx_occupied].normal_x += pcl_vec[j].points[i].normal_x;
					voxel_normal->points[idx_occupied].normal_y += pcl_vec[j].points[i].normal_y;
					voxel_normal->points[idx_occupied].normal_z += pcl_vec[j].points[i].normal_z;
					voxel_normal->points[idx_occupied].intensity += 1.0;
				}
	//			cout << " Xidx : " << Xidx << ", Yidx : " << Yidx << ", Zidx : " << Zidx << endl;
	//			cout << "	idx: " << idx << " (" << voxel_CoG->points[idx].x << ", " << voxel_CoG->points[i].y << ", " << voxel_CoG->points[i].z << ")" << endl;
	//			cout << "	(" << cloud_input->points[i].x << ", " << cloud_input->points[i].y << ", " << cloud_input->points[i].z << ")" << endl;
			}
			/*---- 非占有空間 ----*/
			double k_x = pcl_vec[j].points[i].x; //double stride_x = sqrt(k_x*k_x)/voxel_size;
			double k_y = pcl_vec[j].points[i].y; //double stride_y = sqrt(k_y*k_y)/voxel_size;
			double k_z = pcl_vec[j].points[i].z; //double stride_z = sqrt(k_z*k_z)/voxel_size;
			double stride = sqrt(k_x*k_x + k_y*k_y + k_z*k_z)/voxelSize;
			double p_x = k_x / stride;
			double p_y = k_y / stride;
			double p_z = k_z / stride;
			int Xidx_free, Yidx_free, Zidx_free, idx_free;
			do{
				if( ( fabs(p_x) >= (max_x_range/2))  ||
					( fabs(p_y) >= (max_y_range/2))  ||
					( fabs(p_z) >= (max_z_range/2))  )	break;
				Xidx_free = (int)( ((max_x_range/2) + p_x) / voxelSize );
				Yidx_free = (int)( ((max_y_range/2) + p_y) / voxelSize );
				Zidx_free = (int)( ((max_z_range/2) + p_z) / voxelSize );
				idx_free = (int)( Xidx_free + (Yidx_free*(max_x_range/voxelSize)) + (Zidx_free*(max_x_range/voxelSize)*(max_y_range/voxelSize)) );
				if(idx_free>=voxel_size){
					cout << "\x1b[41mやばいよ！\x1b[0m" << endl;
					break;
				}
				if(is_shot[idx_free][j] == -1){
					is_shot[idx_free][j] = 0;
//					cout << "\x1b[41m 0がいるぞ！\x1b[0m" << endl;
				}
				p_x += k_x/stride;
				p_y += k_y/stride;
				p_z += k_z/stride;
			}while( (sqrt(p_x*p_x)<=sqrt(k_x*k_x)) && (sqrt(p_y*p_y)<=sqrt(k_y*k_y)) && (sqrt(p_z*p_z)<=sqrt(k_z*k_z)) );
//cout << "a";
		}
	}
//cout << "finish!"<<endl;
}

void
OVG::calc_nowPC_isShot(const sensor_msgs::PointCloud2::Ptr cloud_raw){
//cout << "calc_nowPC_isShot" << endl;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::fromROSMsg(*cloud_raw, *cloud);
	pcl::copyPointCloud(*cloud, *cloud_idx);	// 格納されるボクセルグリッドの要素番号をcloud_idx->points.intensityに格納
	is_shot_nowPC = vector<int>(voxel_CoG->points.size(), -1);
	int free_count = 0;
	int occupied_count = 0;
	
	int voxel_size = voxel_CoG->points.size();
	size_t cloud_size = cloud->points.size();
	for(size_t i=0; i<cloud_size; i++){
		if( ( fabs(cloud->points[i].x) <= max_x_range/2.0 ) &&
			( fabs(cloud->points[i].y) <= max_y_range/2.0 ) &&
			( fabs(cloud->points[i].z) <= max_z_range/2.0 )){
				int Xidx_occupied = ( ((max_x_range/2) + cloud->points[i].x) / voxelSize );
				int Yidx_occupied = ( ((max_y_range/2) + cloud->points[i].y) / voxelSize );
				int Zidx_occupied = ( ((max_z_range/2) + cloud->points[i].z) / voxelSize );
				int idx_occupied = (int)( Xidx_occupied + (Yidx_occupied*(max_x_range/voxelSize)) + (Zidx_occupied*(max_x_range/voxelSize)*(max_y_range/voxelSize)) );
				if(idx_occupied<voxel_size){
					if(is_shot_nowPC[idx_occupied] != 1){
						is_shot_nowPC[idx_occupied] = 1;
						occupied_count++;
					}
					cloud_idx->points[i].intensity = (float)idx_occupied;
				}
				//cout << " Xidx : " << Xidx_occupied << ", Yidx : " << Yidx_occupied << ", Zidx : " << Zidx_occupied << endl;
				//cout << "	idx: " << idx_occupied << " (" << voxel_CoG->points[idx_occupied].x << ", " << voxel_CoG->points[idx_occupied].y << ", " << voxel_CoG->points[idx_occupied].z << ")" << endl;
		}
		float k_x = cloud->points[i].x; //double stride_x = sqrt(k_x*k_x)/voxel_size;
		float k_y = cloud->points[i].y; //double stride_y = sqrt(k_y*k_y)/voxel_size;
		float k_z = cloud->points[i].z; //double stride_z = sqrt(k_z*k_z)/voxel_size;
		float stride = sqrt(k_x*k_x + k_y*k_y + k_z*k_z)/voxelSize;	//刻み回数
		float p_x = k_x / stride;
		float p_y = k_y / stride;
		float p_z = k_z / stride;
		int Xidx_free, Yidx_free, Zidx_free, idx_free;
		do{
			if( ( fabs(p_x) >= (max_x_range/2))  ||
				( fabs(p_y) >= (max_y_range/2))  ||
				( fabs(p_z) >= (max_z_range/2))  )	break;
			Xidx_free = (int)( ((max_x_range/2) + p_x) / voxelSize );
			Yidx_free = (int)( ((max_y_range/2) + p_y) / voxelSize );
			Zidx_free = (int)( ((max_z_range/2) + p_z) / voxelSize );
			idx_free = (int)( Xidx_free + (Yidx_free*(max_x_range/voxelSize)) + (Zidx_free*(max_x_range/voxelSize)*(max_y_range/voxelSize)) );
			if(idx_free>=voxel_size){
				cout << "やばいよ！" << endl;
				break;
			}
			if(is_shot_nowPC[idx_free] == -1){
				free_count++;
				is_shot_nowPC[idx_free] = 0;
			}
			p_x += k_x/stride;
			p_y += k_y/stride;
			p_z += k_z/stride;
		}while( (sqrt(p_x*p_x)<=sqrt(k_x*k_x)) && (sqrt(p_y*p_y)<=sqrt(k_y*k_y)) && (sqrt(p_z*p_z)<=sqrt(k_z*k_z)) );
	}
//cout << "\x1b[41m Free count : " << free_count << ", Occupied count : " << occupied_count << "\x1b[0m" << endl;
}

void
OVG::supplement(int iteration){
	int X = max_x_range/voxelSize;
	int Y = max_y_range/voxelSize;
	int Z = max_z_range/voxelSize;
	int supplement_count = 0;
	int iteration_count = 0;

	int vec_size = (int)is_shot_nowPC.size();
	while(iteration_count <= iteration){
		for(int i=0; i<vec_size; i++){
			if(is_shot_nowPC[i] != 1){	//点が存在しない空間
				// iがGridの角だった場合
				if( (i%(X*Y))/X==0 || (i%(X*Y))%X==0 || (i%(X*Y))%X==X-1 || (i%(X*Y))%(X*(Y-1))/X== 0 || i/(X*Y)==0 || i/(X*Y) == Z-1){
					continue;
				}else{	// それ以外の場合
					// iを中心としたX-Y平面の周囲8グリッド
					int xy = (int)is_shot_nowPC[i-X-1] + (int)is_shot_nowPC[i-X] + (int)is_shot_nowPC[i-X+1] + 
							 (int)is_shot_nowPC[i-1] + (int)is_shot_nowPC[i+1] + 
							 (int)is_shot_nowPC[i+X-1] + (int)is_shot_nowPC[i+X] + (int)is_shot_nowPC[i+X+1];
					
					// iを中心としたY-Z平面の周囲8グリッド
					int yz = (int)is_shot_nowPC[(i-X*Y)-X] + (int)is_shot_nowPC[i-X*Y] + (int)is_shot_nowPC[(i-X*Y)+X] +
							 (int)is_shot_nowPC[i-X] + (int)is_shot_nowPC[i+X] +
							 (int)is_shot_nowPC[(i+X*Y)-X] + (int)is_shot_nowPC[i+X*Y] + (int)is_shot_nowPC[(i+X*Y)+X];

					// iを中心としたZ-X平面の周囲8グリッド
					int zx = (int)is_shot_nowPC[(i-X*Y)-1] + (int)is_shot_nowPC[i-X*Y] + (int)is_shot_nowPC[(i-X*Y)+1] + 
							 (int)is_shot_nowPC[i-1] + (int)is_shot_nowPC[i+1] + 
							 (int)is_shot_nowPC[(i+X*Y)-1] + (int)is_shot_nowPC[i+X*Y] + (int)is_shot_nowPC[(i+X*Y)+1];

					if(xy>=5 || yz>=5 || zx>=5){
						is_shot_nowPC[i] = 1;
						supplement_count++;
					}
				}
			}else{	//点が存在する空間
				// iがGridの角だった場合
				if( (i%(X*Y))/X==0 || (i%(X*Y))%X==0 || (i%(X*Y))%X==X-1 || (i%(X*Y))%(X*(Y-1))/X== 0 || i/(X*Y)==0 || i/(X*Y) == Z-1){
					continue;
				}else{	// それ以外の場合
					int xy = (int)is_shot_nowPC[i-X-1] + (int)is_shot_nowPC[i-X] + (int)is_shot_nowPC[i-X+1] + 
							 (int)is_shot_nowPC[i-1] + (int)is_shot_nowPC[i+1] + 
							 (int)is_shot_nowPC[i+X-1] + (int)is_shot_nowPC[i+X] + (int)is_shot_nowPC[i+X+1];
					
					int yz = (int)is_shot_nowPC[(i-X*Y)-X] + (int)is_shot_nowPC[i-X*Y] + (int)is_shot_nowPC[(i-X*Y)+X] +
							 (int)is_shot_nowPC[i-X] + (int)is_shot_nowPC[i+X] +
							 (int)is_shot_nowPC[(i+X*Y)-X] + (int)is_shot_nowPC[i+X*Y] + (int)is_shot_nowPC[(i+X*Y)+X];

					int zx = (int)is_shot_nowPC[(i-X*Y)-1] + (int)is_shot_nowPC[i-X*Y] + (int)is_shot_nowPC[(i-X*Y)+1] + 
							 (int)is_shot_nowPC[i-1] + (int)is_shot_nowPC[i+1] + 
							 (int)is_shot_nowPC[(i+X*Y)-1] + (int)is_shot_nowPC[i+X*Y] + (int)is_shot_nowPC[(i+X*Y)+1];

					if(xy<=1 && yz<=1 && zx<=1){
						is_shot_nowPC[i] = 0;
					}
				}
			}
		}
		iteration_count++;
	}
	cout << "Total supplement : " << supplement_count << endl;
}

bool
OVG::getPose(geometry_msgs::Pose& msg, ros::Time t){
//cout << "getPose" << endl;
	geometry_msgs::PoseStamped pose_tmp;
	init_pose.header.stamp = t;
	try{
		tflistener.waitForTransform("/odom", "/base_link", t, ros::Duration(1.0));
		tflistener.transformPose("/odom", t, init_pose, "/base_link", pose_tmp);
		msg = pose_tmp.pose;
		return true;
	}catch(tf::TransformException ex){
		ROS_ERROR("%s\n", ex.what());
		return false;
	}
}

void
OVG::broad_tf(original_msgs::pointsWithPose msg){
	tf_.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
	tf_.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
	tfbroadcaster_.sendTransform(tf::StampedTransform(tf_, msg.points.header.stamp, std::string("/odom"), std::string("tgt") ));
}


void
OVG::save_rawData(const sensor_msgs::PointCloud2::Ptr cloud_raw){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	static int idx;
	char file_name[100] = {};

	pcl::fromROSMsg(*cloud_raw, *cloud);
	sprintf(file_name, "/home/amsl/ros_catkin_ws/src/pcd_data/v4/cloud_raw/%d.pcd", idx++);
	pcl::io::savePCDFileBinary(file_name, *cloud);
	cout << "save " << file_name << endl;
}
