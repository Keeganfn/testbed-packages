#include "ros/ros.h" 
#include "messages/TestInfo.h"
#include "std_msgs/String.h"
class PubSub_bed{

	public:
		//creates publishers subscribers
		PubSub_bed(ros::NodeHandle& n){
			pub_control = n.advertise<messages::TestInfo>("cycle_complete", 1000);
			pub_arduino = n.advertise<messages::TestInfo>("reset", 1000);
			sub_arm = n.subscribe<messages::TestInfo>("arm_complete", 10, &PubSub_bed::arm_callback, this);
			sub_arduino = n.subscribe<messages::TestInfo>("reset_complete", 10, &PubSub_bed::arduino_callback, this);
		}
		
		//when "arm_complete" topic is published to from the arm_communication node this is called
		void arm_callback(const messages::TestInfo::ConstPtr& msg){
			ROS_INFO("I heard from arm_complete: ANGLE(%d), NUM_CYCLES(%d), ERROR(%d)",
			(int)msg->rotation, (int)msg->number_of_cycles, (int)msg->status);

			ROS_INFO("SENDING TESTBED RESET MESSAGE");
			message = *msg;
			publish_arduino(message);
		}
		//when "reset_complete" topic is published to from the arduino this is called
		void arduino_callback(const messages::TestInfo::ConstPtr& msg){	
			ROS_INFO("I heard from reset_complete: ANGLE(%d), NUM_CYCLES(%d), ERROR(%d)",
		 (int)msg->rotation, (int)msg->number_of_cycles, (int)msg->status);
			ROS_INFO("TESTBED RESET COMPLETE");
			ROS_INFO("PUBLISHING COMPLETE TO CONTROL NODE");
			message = *msg;
			publish_complete(message);
		}

		//publishes "reset" topic to arduino which lets the testbed know to begin resetting
		void publish_arduino(messages::TestInfo& msg){
      while (0 == pub_arduino.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
      }   
			pub_arduino.publish(msg);
		}

		//publishes to start_tests node that 1 cycle of this test is complete on the "cycle_complete" topic
		void publish_complete(messages::TestInfo& msg){
      while (0 == pub_control.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
      }   
			pub_control.publish(msg);
		}

	private:
			ros::Publisher pub_control;
			ros::Publisher pub_arduino;
			ros::Subscriber sub_arm;
			ros::Subscriber sub_arduino;
			messages::TestInfo message;
};

int main(int argc, char **argv){
	//setting up node
	ros::init(argc, argv, "bed_reset");
	ros::NodeHandle n;
	//creates PubSub for all topics and comms necessary
	PubSub_bed pub_obj(n);

	ros::spin();
	return 0;

}




