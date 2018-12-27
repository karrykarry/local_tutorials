#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include<dynamic_reconfigure/server.h>
#include<local_tutorials/d_kan_gridConfig.h>


using namespace std;	

class Pcd2Grid{
	private:
		dynamic_reconfigure::Server<local_tutorials::d_kan_gridConfig> server;
		ros::Publisher map_pub;
		ros::Publisher grid_pub;
		ros::Publisher cell_pub;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN;
	
		int cell_filter;
		float width,height,R;

	public:
		Pcd2Grid(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void map_read(string filename);
		void setparam(nav_msgs::OccupancyGrid *map, nav_msgs::GridCells *obstacle_cell);
		void create_obstacle_map(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, nav_msgs::OccupancyGrid *map_, nav_msgs::GridCells *obstacle_cell_);
		void pub_data(nav_msgs::OccupancyGrid map, nav_msgs::GridCells obstacle_cell);
		void paramcallback(local_tutorials::d_kan_gridConfig &config, uint32_t level);
};

Pcd2Grid::Pcd2Grid(ros::NodeHandle n,ros::NodeHandle priv_nh) :
	cloud_IN(new pcl::PointCloud<pcl::PointXYZI>),
	cell_filter(30),width(60),height(30),R(0.1)
{
	server.setCallback(boost::bind(&Pcd2Grid::paramcallback, this, _1, _2));
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_map", 10);
	grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/grid_map", 10);
	cell_pub = n.advertise<nav_msgs::GridCells>("/cell_map", 10);

}

void
Pcd2Grid::map_read(string filename){
	if( pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_IN, pc);

	pc.header.frame_id  = "map";	
	pc.header.stamp  = ros::Time::now();
	map_pub.publish(pc);

	nav_msgs::OccupancyGrid map;
	nav_msgs::GridCells obstacle_cell;

	setparam(&map,&obstacle_cell);
	create_obstacle_map(cloud_IN,&map,&obstacle_cell);
	pub_data(map,obstacle_cell);


}

void 
Pcd2Grid::paramcallback(local_tutorials::d_kan_gridConfig &config, uint32_t level) {
	 ROS_INFO("Reconfigure Request: %d %lf %lf %lf", 
	 		config.cell_filter, config.width, config.height, config.resolution);

	cell_filter = config.cell_filter;
	width = config.width;
	height = config.height;
	R = config.resolution;
	
	nav_msgs::OccupancyGrid map;
	nav_msgs::GridCells obstacle_cell;

	setparam(&map,&obstacle_cell);
	create_obstacle_map(cloud_IN,&map,&obstacle_cell);
	pub_data(map,obstacle_cell);
}

void
Pcd2Grid::setparam(nav_msgs::OccupancyGrid *map, nav_msgs::GridCells *obstacle_cell){
	map->header.stamp = ros::Time::now();
   	map->header.frame_id = "/map";
	map->data.resize(int(width/R) * int(height/R));
	map->info.width = int(width/R);
	map->info.height = int(height/R);
	map->info.resolution = R;
	map->info.origin.position.x = (0.0 - width)/2.0; 
	map->info.origin.position.y = (0.0 - height)/2.0;

	obstacle_cell->header.stamp = ros::Time::now();
	obstacle_cell->header.frame_id = "/map";
	obstacle_cell->cell_width = R;
	obstacle_cell->cell_height = R;
}


void 
Pcd2Grid::create_obstacle_map(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, nav_msgs::OccupancyGrid *map_, nav_msgs::GridCells *obstacle_cell_){            //全体のマップを作成
	geometry_msgs::Point obstacle_point;

	vector<int>	count((long(width/R) * long(height/R)), 0);
	for(int i = 0; i != map_->data.size(); i++){
		map_->data[i] = 0;
	}


	for(size_t i = 0; i < cloud->points.size(); i++){
		int x = int((cloud->points[i].x - map_->info.origin.position.x) / R);
		int y = int((cloud->points[i].y - map_->info.origin.position.y) / R);
		if((0 <= x && x < width/R) && (0 <= y && y < height/R)){
			long num = x + y * map_->info.width;
			count[num] += 1; 
		}
	}

	cout<<width/R<<endl;
	cout<<count.size()<<endl;
	for(int i = 0;i<count.size(); i++){
	// for(int i = 0;i<12000; i++){
		if(count[i] > cell_filter){
			map_->data[i] = 100;
	
			obstacle_point.x = (i % map_->info.width) * R + map_->info.origin.position.x;
			obstacle_point.y = int(i / map_->info.width) * R + map_->info.origin.position.y;
			obstacle_point.z = 0;
			
			obstacle_cell_->cells.push_back(obstacle_point);	
		}
	}

}

void 
Pcd2Grid::pub_data(nav_msgs::OccupancyGrid map, nav_msgs::GridCells obstacle_cell){
	grid_pub.publish(map);
	cell_pub.publish(obstacle_cell);
}

int main (int argc, char** argv){
	ros::init(argc,argv,"pcd2grid_map");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	Pcd2Grid pcd2grid(n,priv_nh);
	string file = argv[1];
	
	pcd2grid.map_read(file.c_str());

	cout <<"----- grid_pub ok ------" <<endl;

	ros::spin();

	return 0;
}


