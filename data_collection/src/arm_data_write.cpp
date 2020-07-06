#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/String.h>
#include <stdlib.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_data_write");
	system("rosbag record -a -o ~/testbed/src/data_collection/src/bagfile_data/test __name:=my_bag");
	ros::spin();
}
