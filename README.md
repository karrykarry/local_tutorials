# .local_tutorials

## Requirement
- ros (kinetic)
- PCL 1.8


## Install local_tutorials ros-pkg in your src folder  of your Catkin workspace. 

```shell
cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/velodyne
cd ~/catkin_ws
catkin_make
```
### How to use

1. Run odometer.The requested topic is /odom(wheel encoder) /imui/data(gyro sensor)
     ```
     ~$ roslaunch local_tutorials odometer.launch
	 ```
	 
