//=====================================================================
//  Author: Raúl Cebolla Arroyo
//  File:
//  Version:
//  Description:
//  Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <string>
#include <vector>
#include <clhero_gait_controller/GaitPatternControl.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern
#define INVALID_COMMAND_ERR 147
#define EXIT_COMMAND 148
#define HELP_COMMAND 149

#define REGISTERED_GP_PARAM_NAMESPACE "/clhero/gait_patterns/registered"
#define CURRENT_GP_PARAM_NAMESPACE "/clhero/gait_patterns/current"

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

ros::ServiceClient client;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Declare the functions needed

int readCommandLine (){
	
	std::string buffer;
	int int_buff = 0;

	std::getline(std::cin, buffer);
	//std::cin >> buffer;

	if((buffer.compare("q") == 0)||(buffer.compare("Q") == 0)){
		return EXIT_COMMAND;
	}else if((buffer.compare("h") == 0)||(buffer.compare("H") == 0)){
		return HELP_COMMAND;
	}else{
		try{
			int_buff = std::stoi(buffer, NULL);
		}catch(const std::invalid_argument& ia){
			int_buff = INVALID_COMMAND_ERR;
		}
		return int_buff;
	}
}

bool readYN (){
	
	std::string buffer;
	int int_buff = 0;

	std::getline(std::cin, buffer);
	//std::cin >> buffer;

	if((buffer.compare("y") == 0)||(buffer.compare("Y") == 0)){
		return true;
	}else if((buffer.compare("n") == 0)||(buffer.compare("N") == 0)){
		return false;
	}else{
		std::cout << "Invalid command." << std::endl;
	}

	return false;
}


void resetRequest (clhero_gait_controller::GaitPatternControl& msg){
	msg.request.pattern_name.clear();
	msg.request.args.clear();
	msg.request.order.clear();
	return;
}

bool sendRequest (clhero_gait_controller::GaitPatternControl& msg){
	
	std::cout << "Sending request to gait pattern controller" << std::endl;
	
	if(client.call(msg)){
		std::cout << "Message successfully sent." << std::endl;
		
		switch(msg.response.ans){
			case 1:
				std::cout << "[Error] " << "Order not recognized" << std::endl;
				break;
			case 2:
				std::cout << "[Error] " << "Gait pattern is not in the list" << std::endl;
				break;
			case 3:
				std::cout << "[Error] " << "Gait pattern name does not match with the current active pattern." << std::endl;
				break;
			case 4:
				std::cout << "[Error] " << "Could not change gait pattern at the command interface" << std::endl;
				break;
			default:
				break;
		}

		return true;

	}else{

		std::cout << "[Error] Could not call server." << std::endl;
		return false;

	}

	return true;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main (int argc, char** argv){

	//----------------------------------------------------
	//    ROS starting statements
	//----------------------------------------------------

	ros::init(argc, argv, "pattern_control_terminal");
	ros::NodeHandle nh;

	client = nh.serviceClient<clhero_gait_controller::GaitPatternControl>("/clhero_gait_control/gait_pattern_control");

	//----------------------------------------------------
	//    GUI 
	//----------------------------------------------------

	bool active = true;
	bool show_main_menu = true;
	bool intro_pattern_active = true;

	clhero_gait_controller::GaitPatternControl msg;

	int command, pattern_choice;
	std::vector<std::string> pattern_names;
	std::string current_pattern_name;

	std::cout << "------------------------------------------------------------" << std::endl;
	std::cout << "	Terminal for clhero gait control " << std::endl;
	std::cout << "------------------------------------------------------------" << std::endl;

	while(active && ros::ok()){

		if(show_main_menu){
			//Show the different actions
			std::cout << "Select a command: " << std::endl;
			std::cout << "  [1] Start gait pattern" << std::endl;
			std::cout << "  [2] Stop gait pattern" << std::endl;
			std::cout << "  [3] Pause gait pattern execution" << std::endl;
			std::cout << "  [4] Continue paused execution" << std::endl;
			std::cout << "  [5] Force gait state" << std::endl;
			std::cout << "  [6] Update movement arguments" << std::endl;
			std::cout << "  [h] Show main menu" << std::endl;
			std::cout << "  [q] Exit" << std::endl;

			show_main_menu = false;
		}
		
		std::cout << "------------------------------------" << std::endl;
		std::cout << "  [Command]: "; command = readCommandLine();
		std::cout << "------------------------------------" << std::endl;

		switch (command){
			case 1:

				std::cout << "-- " << "Start gait pattern" << " --" << std::endl;

				msg.request.order = "start";

				nh.getParam(REGISTERED_GP_PARAM_NAMESPACE, pattern_names);

				if(pattern_names.size() < 1){
					std::cout << "[Error] " << "There are no avaiable patterns" << std::endl;
					break;
				}

				std::cout << "Available patterns: " << std::endl;

				for(int i = 0; i < pattern_names.size(); i++){
					std::cout << "  [" << i+1 << "] " << pattern_names[i] << std::endl;
				}

				intro_pattern_active = true;

				while(intro_pattern_active){

					std::cout << "Introduce the desired pattern to start (press q to cancel): ";
					pattern_choice = readCommandLine();

					switch(pattern_choice){
						case EXIT_COMMAND: 
							intro_pattern_active = false;
							break;
						case HELP_COMMAND:
							std::cout << "Invalid command." << std::endl;
							break;
						case INVALID_COMMAND_ERR:
							std::cout << "Invalid command." << std::endl;
							break;
						default:
							if((pattern_choice < 1) || (pattern_choice > pattern_names.size())){
								std::cout << "Number out of range." << std::endl;
								break;
							}else{
								msg.request.pattern_name = pattern_names[pattern_choice-1];
								std::cout << "Starting pattern: " << msg.request.pattern_name << std::endl;
								sendRequest(msg);
								intro_pattern_active = false;
								break;
							}
					}

				}

				break;

			case 2:

				std::cout << "-- " << "Stop gait pattern" << " --" << std::endl;

				msg.request.order = "stop";

				nh.getParam(CURRENT_GP_PARAM_NAMESPACE, current_pattern_name);
				std::cout << "Current active pattern: " << current_pattern_name << std::endl;
				std::cout << "Do you want to stop current active pattern? [y/n] ";

				if(readYN()){
					msg.request.pattern_name = current_pattern_name;
					sendRequest(msg);
				}else{
					std::cout << "Stop cancelled."<< std::endl;
				}

				break;

			case 3:

				std::cout << "-- " << "Pause gait pattern" << " --" << std::endl;

				msg.request.order = "pause";

				nh.getParam(CURRENT_GP_PARAM_NAMESPACE, current_pattern_name);
				std::cout << "Current active pattern: " << current_pattern_name << std::endl;
				std::cout << "Do you want to pause current active pattern? [y/n] ";

				if(readYN()){
					msg.request.pattern_name = current_pattern_name;
					sendRequest(msg);
				}else{
					std::cout << "Pause cancelled."<< std::endl;
				}

				break;

			case 4:

				std::cout << "-- " << "Resume gait pattern" << " --" << std::endl;

				msg.request.order = "continue";

				nh.getParam(CURRENT_GP_PARAM_NAMESPACE, current_pattern_name);
				std::cout << "Current active pattern: " << current_pattern_name << std::endl;
				std::cout << "Do you want to resume current active pattern? [y/n] ";

				if(readYN()){
					msg.request.pattern_name = current_pattern_name;
					sendRequest(msg);
				}else{
					std::cout << "Resume cancelled."<< std::endl;
				}

				break;

			case 5:

				std::cout << "-- " << "Force state" << " --" << std::endl;

				msg.request.order = "force_state";

				break;

			case 6:

				std::cout << "-- " << "Update arguments" << " --" << std::endl;

				msg.request.order = "update_args";

				nh.getParam(REGISTERED_GP_PARAM_NAMESPACE, pattern_names);

				if(pattern_names.size() < 1){
					std::cout << "[Error] " << "There are no avaiable patterns" << std::endl;
					break;
				}

				std::cout << "Available patterns: " << std::endl;

				for(int i = 0; i < pattern_names.size(); i++){
					std::cout << "  [" << i+1 << "] " << pattern_names[i] << std::endl;
				}

				intro_pattern_active = true;

				while(intro_pattern_active){

					std::cout << "Introduce the desired pattern to update arguments (press q to cancel): ";
					pattern_choice = readCommandLine();

					switch(pattern_choice){
						case EXIT_COMMAND: 
							intro_pattern_active = false;
							break;
						case HELP_COMMAND:
							std::cout << "Invalid command." << std::endl;
							break;
						case INVALID_COMMAND_ERR:
							std::cout << "Invalid command." << std::endl;
							break;
						default:
							if((pattern_choice < 1) || (pattern_choice > pattern_names.size())){
								std::cout << "Number out of range." << std::endl;
								break;
							}else{
								msg.request.pattern_name = pattern_names[pattern_choice-1];
								intro_pattern_active = false;
								break;
							}
					}

				}

				if(pattern_choice == EXIT_COMMAND){
					std::cout << "Update cancelled." << std::endl;
					break;
				}

				std::cout << "Introduce the arguments as:" << std::endl;
				std::cout << "  (e.g): key_1: value_1, key_2: value_2, key_3: value_3" << std::endl;
				std::cout << "Arguments: " << std::endl;

				std::getline(std::cin, msg.request.args);
				sendRequest(msg);

				break;

			case HELP_COMMAND:

				show_main_menu = true;

				break;

			case EXIT_COMMAND:

				std::cout << "Exiting terminal process." << std::endl;
				active = false;

				break;

			default:
					
				std::cout << "Invalid command." << std::endl;
				
				break;
		}

		resetRequest(msg);

	}

	return 0;

}