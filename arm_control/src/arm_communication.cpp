#include "ros/ros.h"
#include "messages/TestInfo.h"
#include "std_msgs/String.h"
#include <sstream>

class PubSub_arm{

  public:
    //creates publisher and subscribers necessary
    PubSub_arm(ros::NodeHandle& n){
      pub_bed = n.advertise<messages::TestInfo>("arm_complete", 1000);
			pub_arm = n.advertise<std_msgs::String>("start_arm", 1000);
			sub_bed = n.subscribe<messages::TestInfo>("cycle_start", 1000, &PubSub_arm::callback, this);
			sub_arm = n.subscribe<std_msgs::String>("arm_done", 1000, &PubSub_arm::arm_callback, this);
    }

		//this callback will call on the necessary arm functions in the future
		void callback(const messages::TestInfo::ConstPtr& msg){
  		ROS_INFO("I heard from cycle_start test ANGLE(%d), NUM_CYCLES(%d), ERROR(%d)",
		 (int)msg->rotation, (int)msg->number_of_cycles, (int)msg->status);

    	ROS_INFO("Sending kinova arm start message");
			message = *msg;
			publish_arm();
		}

		//this callback will call on the necessary arm functions in the future
		void arm_callback(const std_msgs::String::ConstPtr& msg){
  	  ROS_INFO("ARM MANUEVER FINISHED");
    	ROS_INFO("PUBLISHING TO BED");
			publish_bed(message);
		}


    //publishes to kinova node that it needs to start
    void publish_arm(){
      while (0 == pub_arm.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
      }   
			std_msgs::String arm_message;
			arm_message.data = "start";
      pub_arm.publish(arm_message);
    }

    //publishes to bed node that all processes are complete
    void publish_bed(messages::TestInfo& msg){
      while (0 == pub_bed.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
      }   
      pub_bed.publish(msg);
    }

  private:
  	ros::Publisher pub_arm;
  	ros::Publisher pub_bed;
		ros::Subscriber sub_bed;
		ros::Subscriber sub_arm;
		messages::TestInfo message;
};

int main(int argc, char **argv){
  //setting up node
  ros::init(argc, argv, "arm_communication");
  ros::NodeHandle n;

  //creates PubSub which starts all subscribers and publishers
  PubSub_arm pub_obj(n);
  ros::spin();
  return 0;



}

