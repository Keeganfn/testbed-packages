#include "ros/ros.h"
#include "messages/TestInfo.h"
#include "messages/TestInfoArray.h"
#include <iostream>
#include <string>
#include <sstream>

int cycle_input();
int angle_input();
int enter_again();


int main(int argc, char** argv){
	//initializes node
	ros::init(argc, argv, "test_queue");
	ros::NodeHandle n;

	int cycle_num = 0;
	int angle = 0;
	int again = 1;
	messages::TestInfo temp;	
	messages::TestInfoArray request_list;
	request_list.queue_size = 0;
	
	//gets user input for number of tests, cycles and angles
	while(again == 1){
		cycle_num = cycle_input();
		if(cycle_num != 0){
			temp.number_of_cycles = cycle_num; 
			angle = angle_input();
			temp.rotation = angle;
			temp.status = 0;
			request_list.queue.push_back(temp);
			request_list.queue_size += 1;
		}
		again = enter_again();
	}	
	
	//initializes publisher and waits until subscribers have connected to "request_queue" topic
	ros::Publisher pub = n.advertise<messages::TestInfoArray>("request_queue", 1000);
	while (0 == pub.getNumSubscribers()) {
		ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(0.1).sleep();
  }

//lets user know what tests were passed along then published
for(int i = 0; i < request_list.queue_size; i++){
	ROS_INFO("Cycles: %d, Rotation: %d, Size of Queue: %d", int(request_list.queue[i].number_of_cycles), int(request_list.queue[i].rotation), int(request_list.queue_size));
 }
	pub.publish(request_list);
	ros::spin();
	
	return 0;
}
	
//Does error checking on user input for if they would like to enter another test
int enter_again(){
	int final_num = 0;
	int sign = 0;
	std::string temp;
	bool check = true;
	
	while(check){
		check = false;
		std::cout << "Would you like to enter another test? (1 = yes, 0 = no): " << std::endl;
		std::getline(std::cin, temp);

		for(int i = 0; i < temp.size(); i++){
			if(!(temp.at(i) >= '0' && temp.at(i) <= '1')){	
				std::cout << "You entered something besides a 1 or 0, Try again \n" << std::endl;
				check = true;
				break;
			}
		}
	
		if(check == false){
			std::stringstream convert(temp);
			convert >> final_num;
			if(!(final_num == 1 || final_num == 0)){
				std::cout << "Your integer needs to be either 1 or 0, Try again\n" << std::endl;
				check = true;
			}
		}
	}		
	return final_num;		

}

//Does error checking for user input on number of cycles
int cycle_input(){
	int final_num = 0;
	int sign = 0;
	std::string temp;
	bool check = true;
	
	while(check){
		check = false;
		std::cout << "How many cycles of this test would you like to run? (Please enter an integer >= 1 or 0 to cancel this test): " << std::endl;
		std::getline(std::cin, temp);

		for(int i = 0; i < temp.size(); i++){
			if(!(temp.at(i) >= '0' && temp.at(i) <= '9')){	
				std::cout << "You entered something besides an integer, Try again \n" << std::endl;
				check = true;
				break;
			}
		}
	
		if(check == false){
			std::stringstream convert(temp);
			convert >> final_num;
			if(!(final_num >= 0)){
				std::cout << "Your integer needs to be within the range -180 <= x <= 180, Try again\n" << std::endl;
				check = true;
			}
		}
	}		
	return final_num;		

}
//Does error checking on input for angle used for tests
int angle_input(){
	int final_num = 0;
	int sign = 0;
	std::string temp;
	bool check = true;
	
	while(check){
		check = false;
		std::cout << "What angle would you like the object to reset at? (Please enter integer within range -180 <= x <= 180): " << std::endl;
		std::getline(std::cin, temp);
		sign = 0;

		for(int i = 0; i < temp.size(); i++){
			if(temp.at(i) == '-' && i > 0){
				sign = 2;
			}
			if(!((temp.at(i) >= '0' && temp.at(i) <= '9') || (temp.at(i) == '-') && sign <= 1)){	
				std::cout << "You entered something besides an integer, Try again \n" << std::endl;
				check = true;
				break;
			}
		}
	
		if(check == false){
			std::stringstream convert(temp);
			convert >> final_num;
			if(!(final_num <= 180 && final_num >= -180)){
				std::cout << "Your integer needs to be within the range -180 <= x <= 180, Try again\n" << std::endl;
				check = true;
			}
		}
	}		
	return final_num;		

}


