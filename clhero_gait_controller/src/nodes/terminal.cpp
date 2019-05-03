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
#define REGISTERED_GP_PARAM_NAMESPACE "/clhero/gait_patterns/registered"
#define CURRENT_GP_PARAM_NAMESPACE "/clhero/gait_patterns/current"

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Declare the functions needed

int readCommandLine (){
	
	std::string buffer;
	int int_buff = 0;

	std::getline(std::cin, buffer);
	//std::cin >> buffer;

	if(buffer.compare("q") == 0){
		return -1;
	}else{
		try{
			int_buff = std::stoi(buffer, NULL);
		}catch(const std::invalid_argument& ia){
			int_buff = INVALID_COMMAND_ERR;
		}
		return int_buff;
	}
}

void resetRequest (clhero_gait_controller::GaitPatternControl& msg){
	msg.request.pattern_name.clear();
	msg.request.args.clear();
	msg.request.order.clear();
	return;
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

	ros::ServiceClient client = nh.serviceClient<clhero_gait_controller::GaitPatternControl>("gait_pattern_control");

	//----------------------------------------------------
	//    GUI 
	//----------------------------------------------------

	bool active = true;
	bool show_main_menu = true;

	clhero_gait_controller::GaitPatternControl msg;

	int command;
	std::vector<std::string> pattern_names;

	std::cout << "------------------------------------------------------------" << std::endl;
	std::cout << "	Terminal for clhero gait control " << std::endl;
	std::cout << "------------------------------------------------------------" << std::endl;

	while(active){

		if(show_main_menu){
			//Show the different actions
			std::cout << "Select a command: " << std::endl;
			std::cout << "  [1] Start gait pattern" << std::endl;
			std::cout << "  [2] Stop gait pattern" << std::endl;
			std::cout << "  [3] Pause gait pattern execution" << std::endl;
			std::cout << "  [4] Continue paused execution" << std::endl;
			std::cout << "  [5] Force gait state" << std::endl;
			std::cout << "  [6] Update movement arguments" << std::endl;
			std::cout << "  [q] Exit" << std::endl;

			show_main_menu = false;
		}
		
		std::cout << "------------------------------------" << std::endl;
		std::cout << "  [Command]: ";

		command = readCommandLine();

		switch (command){
			case 1:

				std::cout << "-- " << "Start gait pattern" << " --" << std::endl;

				msg.request.order = "start";

				nh.getParam(REGISTERED_GP_PARAM_NAMESPACE, pattern_names);

				if(pattern_names.size() < 1){
					std::cout << "[Error] " << "There are no avaiable patterns" << std::endl;
					break;
				}

				std::cout << "Avaiable patterns: " << std::endl;

				for(int i = 0; i < pattern_names.size(); i++){
					std::cout << "  [" << i+1 << "] " << pattern_names[i] << std::endl;
				}

				break;

			case 2:

				std::cout << "-- " << "Stop gait pattern" << " --" << std::endl;

				break;

			case 3:

				std::cout << "-- " << "Pause gait pattern" << " --" << std::endl;

				break;

			case 4:

				std::cout << "-- " << "Resume gait pattern" << " --" << std::endl;

				break;

			case 5:

				std::cout << "-- " << "Force state" << " --" << std::endl;

				break;

			case 6:

				std::cout << "-- " << "Update arguments" << " --" << std::endl;

				break;

			default:
				if(command < 0){
					std::cout << "Exiting terminal process." << std::endl;
					active = false;
				}else{
					std::cout << "Invalid command." << std::endl;
				}
				break;
		}

		resetRequest(msg);

	}

	return 0;

}