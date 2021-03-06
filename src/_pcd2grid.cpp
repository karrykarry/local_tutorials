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


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
const int cell_filter = 10;		//for ikuta
const float width = 20;
const float height = 20;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.05;                //障害物判定の範囲を指定
ros::Publisher grid_pub;
ros::Publisher cell_pub;

nav_msgs::OccupancyGrid map;
nav_msgs::GridCells obstacle_cell;



void sq_lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::fromROSMsg (*input, *cloud);

	create_obstacle_map(cloud,&map,&obstacle_cell);

	map.header.stamp = ros::Time::now();
   	map.header.frame_id = "/velodyne";
	
	obstacle_cell.header.stamp = ros::Time::now();
	obstacle_cell.header.frame_id = "/velodyne";

	grid_pub.publish(map);
	cell_pub.publish(obstacle_cell);

		
}


void create_obstacle_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid *map, nav_msgs::GridCells *obstacle_cell){            //全体のマップを作成
	geometry_msgs::Point obstacle_point;

	vector<int>	count((long(width/R) * long(height/R)), 0);
	for(int i = 0; i != map->data.size(); i++){
		map->data[i] = 0;
	}

	cout<<map->info.origin.position.x<<endl; 

	for(size_t i = 0; i < cloud->points.size(); i++){
		int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
		int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
		if((0 <= x && x < width/R) && (0 <= y && y < height/R)){
			long num = x + y * map->info.width;
			count[num] += 1; 
		}
	}


	for(int i = 0;i<count.size(); i++){
		if(count[i] > cell_filter){
			map->data[i] = 100;
	
			obstacle_point.x = (i % map->info.width) * R + map->info.origin.position.x;
			obstacle_point.y = int(i / map->info.width) * R + map->info.origin.position.y;
			obstacle_point.z = 0;
			
			obstacle_cell->cells.push_back(obstacle_point);	
		}
	}

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "velo2grid");
	ros::NodeHandle n;

	grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/cloud_grid", 100, true);
	cell_pub = n.advertise<nav_msgs::GridCells>("/cloud_cell", 100, true);

    ros::Subscriber lidar_sub = n.subscribe("/velodyne_obstacles", 1000, sq_lidarCallback);

	map.data.resize(int(width/R) * int(height/R));
	map.info.width = int(width/R);
	map.info.height = int(height/R);
	map.info.resolution = R;
	map.info.origin.position.x = (min_x - width)/2.0; 
	map.info.origin.position.y = (min_y - height)/2.0;

	obstacle_cell.cell_width = R;
	obstacle_cell.cell_height = R;
	

	// pcd2grid(cloud_IN,map);
	
	ros::spin();

	return (0);
}
