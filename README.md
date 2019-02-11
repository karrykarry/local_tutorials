# local_tutorials

## Requirement
- ros (kinetic)
- PCL 1.8


## Install local_tutorials ros-pkg in your src folder  of your Catkin workspace. 

```shell
cd ~/catkin_ws/src/
git clone https://github.com/karrykarry/local_tutorials
cd ~/catkin_ws
catkin_make
```
### How to use

1. Run odometer.The requested topics are /odom(wheel encoder) /imu/data(gyro sensor)
     ```
     ~$ roslaunch local_tutorials odometer.launch
	 ```
	 
2. Run mcl.The requested topics are /odom(wheel encoder) /imu/data(gyro sensor) /velodyne_points(LiDAR).The pcd map is required.
     ```
     ~$ roslaunch local_tutorials mcl.launch
	 ```
