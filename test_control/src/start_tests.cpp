#include "ros/ros.h"
#include "messages/TestInfo.h"
#include "messages/TestInfoArray.h"
#include <stdlib.h>

class Cycle_control{

	public:
		//creates all needed pubs and subs and initializes all vars
		Cycle_control(ros::NodeHandle& n){
			cycle_count = 1;
			test_num = 0;		
			complete = 0;
      pub = n.advertise<messages::TestInfo>("cycle_start", 1000);
			sub = n.subscribe<messages::TestInfoArray>("request_queue", 100, &Cycle_control::callback, this);
		}
		//when "request_queue" topic gets published to by test_queue node this is called 
		void callback(const messages::TestInfoArray::ConstPtr& msg){
				messages::TestInfo temp;
				list.queue_size = msg->queue_size;
				//copies all values from TestInfoArray to class variables
				for(int i = 0; i < msg->queue_size; i++){
					temp = msg->queue[i];
					list.queue.push_back(temp);
				}
				test = list.queue[test_num];
				publish_topic();
		}

		//outputs current test and cycle info
		void current_test(){
			ROS_INFO("Test %d, out of %d.", test_num+1, (int)list.queue_size);	
			ROS_INFO("Cycle %d, out of %d. Angle: %d", cycle_count, (int)test.number_of_cycles, (int)test.rotation);
		}
		
		//publishes current test to "cycle_start" topic where arm_communication is subscribed 
    void publish_topic(){
  		while (0 == pub.getNumSubscribers()) {
    		ROS_INFO("Waiting for subscribers to connect");
    		ros::Duration(0.1).sleep();
  		}
      current_test();
      pub.publish(test);
    }

		//moves to the next cycle/test when bed_reset node is finished
		void next_cycle(){	
			if(cycle_count < test.number_of_cycles){
				cycle_count++;
			}
			else{
				if(test_num < list.queue_size-1){
					test_num++;
					test = list.queue[test_num];
					cycle_count = 1;	
					ROS_INFO("MOVING TO NEXT TEST");
				}
				else{
					ROS_INFO("ALL TESTS COMPLETE");
					complete = 1;
				}
			}
		}

		//checks if all tests have been executed
		int get_complete(){
			return complete;
		}

	private:
		int test_num;
		int cycle_count;
		int complete;
    ros::Publisher pub;
		ros::Subscriber sub;
		messages::TestInfoArray list;
		messages::TestInfo test;
};

//when bed_reset node is complete it publishes to "cycle_complete" topic and calls this function
void call_back(const messages::TestInfo::ConstPtr& msg, Cycle_control* track_cycle){

  ROS_INFO("CYCLE COMPLETE");

	track_cycle->next_cycle();
	if(track_cycle->get_complete() == 0){
		system("rosnode kill /my_bag");
		ros::Duration(.5).sleep();
		system("rosnode kill /data_collector");
		track_cycle->publish_topic();
	}
}

int main(int argc, char** argv){
	//initializes node
	ros::init(argc, argv, "start_tests");
  ros::NodeHandle n;
	Cycle_control track_cycle(n);	
	ros::Subscriber sub = n.subscribe<messages::TestInfo>("cycle_complete", 10, boost::bind(&call_back, _1, &track_cycle));
	ros::spin();
}
