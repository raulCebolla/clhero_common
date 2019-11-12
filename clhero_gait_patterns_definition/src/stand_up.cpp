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
#include <clhero_gait_controller/clhero.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 200
#define PATTERN_NAME "stand_up"
#define PI 3.14159265359
#define LEG_NUMBER 6
#define ANG_THR 0.12217304763960307
#define REV_ANG_THR 3.839724354387525
#define STAND_UP_VEL 2

//----------------------------------------------------
//    Global Variables
//----------------------------------------------------

const std::vector<int> all_legs = {1, 2, 3, 4, 5, 6};
std::unordered_map<std::string,double> movement_parameters;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that updates the parameters
std::unordered_map<std::string, double> update_movement_parameters(std::map<std::string,std::string> args){

	//Gets each of the parameters
	
	//Velocity
	try{
		movement_parameters["velocity"] = std::stod(args.at("velocity"));
	}catch(std::out_of_range& oor){
		movement_parameters["velocity"] = STAND_UP_VEL;
	}

	return movement_parameters;

}

//----------------------------------------------------
//    States
//----------------------------------------------------

//State 1
//Raise the robot into a standing position
void state_1 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Stand up]: Entered state 1");

	update_movement_parameters(clhr->getArgs());
	const double movement_velocity = movement_parameters["velocity"];

	//Check if it's already in position
	std::vector<float> state;
	int legs_in_position = 0;

	state = clhr->getLegsPosition();

	for(int i=0; i < LEG_NUMBER; i++){
		if((fabs(state[i] - 0) < ANG_THR)||(fabs(state[i] - 2*PI) < ANG_THR)){
			legs_in_position++;
		}
	}

	if(legs_in_position == LEG_NUMBER){
		
	}else{
		legs_in_position = 0;
		clhr->setLegPosition(all_legs, 0, movement_velocity);
		clhr->sendCommands();
	}

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 1){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		loop_rate.sleep();
	}

	return;
}

//2nd version of stand up state
//Raise the robot into a standing position
void stand_up_2_state (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Alternating tripod]: Entered state 1");

	std::vector<float> state;
	int legs_in_position = 0;

	update_movement_parameters(clhr->getArgs());
	const double movement_velocity = movement_parameters["velocity"];

	//Check if it's already in position
	state = clhr->getLegsPosition();

	for(int i=0; i < LEG_NUMBER; i++){
		if((fabs(state[i] - 0) < ANG_THR)||(fabs(state[i] - 2*PI) < ANG_THR)){
			legs_in_position++;
		}
	}

	if(legs_in_position == LEG_NUMBER){
		clhr->transition(2);
	}else{
		legs_in_position = 0;
		for(int i=0; i<LEG_NUMBER; i++){
			if(state[i] > REV_ANG_THR){
				clhr->setLegPosition(i+1, 0, movement_velocity); 			
			}else{
				clhr->setLegPosition(i+1, 0, (-0.8)*movement_velocity);
			}
		}
		clhr->sendCommands();
	}

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 1){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		state = clhr->getLegsPosition();

		for(int i=0; i < LEG_NUMBER; i++){
			if((fabs(state[i] - 0) < ANG_THR)||(fabs(state[i] - 2*PI) < ANG_THR)){
				legs_in_position++;
			}
		}

		if(legs_in_position == LEG_NUMBER){
			clhr->transition(2);
		}else{
			legs_in_position = 0;
		}

		loop_rate.sleep();
	}

	return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main (int argc, char** argv){

	//----------------------------------------------------
	//    ROS starting statements
	//----------------------------------------------------

	ros::init(argc, argv, PATTERN_NAME);
	ros::NodeHandle nh;

	//----------------------------------------------------
	//    Gait pattern initialization
	//----------------------------------------------------

	//set the pattern name
	const std::string pattern_name = PATTERN_NAME;

	//Instantiation of the pattern's class
	clhero::Clhero_robot clhr (pattern_name);

	//Register the new gait pattern
	clhero::registerGaitPattern(pattern_name);

	clhr.bufferCommands(true);

	//----------------------------------------------------
	//    State's instantiation
	//----------------------------------------------------

	//attach the states set
	clhr.attachState(1, state_1, STARTING_STATE);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}
