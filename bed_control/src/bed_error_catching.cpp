#include "ros/ros.h" 
#include "std_msgs/Int32.h"


void callback(const std_msgs::Int32::ConstPtr& msg){
		if(msg->data == 100){
			ROS_INFO("Exit Status 100, cone timeout");
		}
		else if(msg->data == 200){
			ROS_INFO("Exit Status 200, reel in timeout");
		}
		else if(msg->data == 300){
			ROS_INFO("Exit Status 300, cone lower fail");
		}
		else if(msg->data == 400){
			ROS_INFO("Exit Status 400, object rotation timeout");
		}
		else{
			ROS_INFO("Exit Status ???, Unknown error occured");
		}
}


int main(int argc, char **argv){
  //setting up node
  ros::init(argc, argv, "bed_error_catching");
  ros::NodeHandle n;
  //creates PubSub for all topics and comms necessary
 	ros::Subscriber error = n.subscribe<std_msgs::Int32>("errors", 1000, callback);	

  ros::spin();
  return 0;

}


