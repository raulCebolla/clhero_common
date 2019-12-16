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
#include <stdexcept>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 100
#define PATTERN_NAME "open_loop_alternating_tripod"
#define PI 3.14159265359
#define GROUND_ANGLE 1.0471975511965976 // 60 [º] (std = 60º)
#define AIR_ANGLE (2*PI - GROUND_ANGLE)
#define GROUND_VELOCITY (1) // aprox 30 [rpm] (std = 1 rad/s)
#define AIR_VELOCITY (5) // (max = 6 rad/s)
#define LEG_NUMBER 6
#define ANG_THR 0.17453292519943295 // 10[º]
#define STAND_UP_VEL 2 
#define REV_ANG_THR 2.792526803190927// 160 [º]
#define MAX_GROUND_ANGLE 3.6208146876268845 // 2*max_take_off_angle
#define MAX_VEL 6.0
#define TIME_CORRECTION_FACTOR 1
#define wait_time_DEFAULT 0.05

//----------------------------------------------------
//    Global Variables
//----------------------------------------------------

//Tripods
const std::vector<int> tripod_1 = {1, 4, 5};
const std::vector<int> tripod_2 = {2, 3, 6};
const std::vector<int> all_legs = {1, 2, 3, 4, 5, 6};

//Movement Parameters
std::unordered_map<std::string, double> movement_parameters;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

std::unordered_map<std::string,double> set_default_movement_parameters(){
	movement_parameters["ground_angle"] = GROUND_ANGLE;
	movement_parameters["air_angle"] = 2*PI - movement_parameters["ground_angle"];
	movement_parameters["ground_velocity"] = GROUND_VELOCITY;
	movement_parameters["air_velocity"] = movement_parameters["air_angle"]/movement_parameters["ground_angle"] * movement_parameters["ground_velocity"];
	movement_parameters["stand_up_velocity"] = STAND_UP_VEL;
	movement_parameters["wait_time"] = wait_time_DEFAULT;
	return movement_parameters;
}

//Function that updates the parameters
std::unordered_map<std::string, double> update_movement_parameters(std::map<std::string,std::string> args){

	//Gets each of the parameters
	
	//Ground angle
	try{

		movement_parameters["ground_angle"] = std::stod(args.at("ground_angle"));

		//checks that the value is inside its limits
		if(movement_parameters["ground_angle"] > MAX_GROUND_ANGLE){
			ROS_INFO("[alternating_tripod]: Ground angle value exceeded, setting to: %f", MAX_GROUND_ANGLE);
			movement_parameters["ground_angle"] = MAX_GROUND_ANGLE;
		}

	}catch(std::out_of_range& oor){}

	//Air angle

	//Due to geometry constraints
	movement_parameters["air_angle"] = 2*PI - movement_parameters["ground_angle"];

	//Ground velocity
	try{
		movement_parameters["ground_velocity"] = std::stod(args.at("ground_velocity"));
	}catch(std::out_of_range& oor){}

	if(movement_parameters["ground_velocity"] > MAX_VEL){
		ROS_INFO("[alternating_tripod]: Ground velocity exceeded, setting to: %f", MAX_VEL);
		movement_parameters["ground_velocity"] = MAX_VEL;
	}

	//Air velocity
	movement_parameters["air_velocity"] = movement_parameters["air_angle"]/movement_parameters["ground_angle"] * movement_parameters["ground_velocity"];

	//If the air velocity is exceeded, sets to its maximum
	if(movement_parameters["air_velocity"] > MAX_VEL){
		ROS_INFO("[alternating_tripod]: Air value exceeded, setting to: %f", MAX_VEL);
		movement_parameters["air_velocity"] = MAX_VEL;
	} 

	//Stand up velocity
	try{
		movement_parameters["stand_up_velocity"] = std::stod(args.at("stand_up_velocity"));
	}catch(std::out_of_range& oor){	}

	//threshold time
	try{
		movement_parameters["wait_time"] = std::stod(args.at("wait_time"));
	}catch(std::out_of_range& oor){	}

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

	ROS_INFO("[Alternating tripod]: Entered state 1");

	update_movement_parameters(clhr->getArgs());

	std::vector<float> state;
	int legs_in_position = 0;
	const double stand_up_vel = movement_parameters["stand_up_velocity"];

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
		clhr->setLegPosition(all_legs, 0, stand_up_vel);
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
	const double stand_up_vel = movement_parameters["stand_up_velocity"];
	const double landing_pos = 2*PI - movement_parameters["ground_angle"]/2;
	const double take_off_pos = movement_parameters["ground_angle"]/2;
	const double ground_velocity = movement_parameters["ground_velocity"];
	const double air_velocity = movement_parameters["air_velocity"];

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
				clhr->setLegPosition(i+1, 0, stand_up_vel); 			
			}else{
				clhr->setLegPosition(i+1, 0, (-0.8)*stand_up_vel);
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

//Estado 2
void state_2 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Alternating tripod]: Entered state 2");

	std::vector<float> state;
	int legs_in_position = 0;

	update_movement_parameters(clhr->getArgs());
	const double stand_up_vel = movement_parameters["stand_up_velocity"];
	const double landing_pos = 2*PI - movement_parameters["ground_angle"]/2;
	const double take_off_pos = movement_parameters["ground_angle"]/2;
	const double ground_velocity = movement_parameters["ground_velocity"];
	const double air_velocity = movement_parameters["air_velocity"];

	/*for(int i=0; i < tripod_1.size(); i++){
		clhr->setLegPosition(tripod_1[i], TAKE_OFF_ANG, GROUND_VELOCITY);
	}*/

	clhr->setLegPosition(tripod_1, take_off_pos, ground_velocity);

	/*for(int i=0; i < tripod_2.size(); i++){
		clhr->setLegPosition(tripod_2[i], LANDING_ANG, (-1.0)*GROUND_VELOCITY);
	}*/

	clhr->setLegPosition(tripod_2, landing_pos, (-2)*ground_velocity);

	//Sends all the commands
	clhr->sendCommands();

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 2){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		state = clhr->getLegsPosition();

		for(int i=0; i < tripod_1.size(); i++){
			if((fabs(state[tripod_1[i]-1] - take_off_pos) < ANG_THR)){
				legs_in_position++;
			}
		}

		for(int i=0; i < tripod_2.size(); i++){
			if((fabs(state[tripod_2[i]-1] - landing_pos) < ANG_THR)){
				legs_in_position++;
			}
		}

		if(legs_in_position == LEG_NUMBER){
			clhr->transition(3);
		}else{
			legs_in_position = 0;
		}

		loop_rate.sleep();
	}

	return;
}

//Estado 3
void state_3 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Alternating tripod]: Entered state 3");

	std::vector<float> state;
	int legs_in_position = 0;

	double time_in_state = 0.0;

	update_movement_parameters(clhr->getArgs());
	const double stand_up_vel = movement_parameters["stand_up_velocity"];
	const double landing_pos = 2*PI - movement_parameters["ground_angle"]/2;
	const double take_off_pos = movement_parameters["ground_angle"]/2;
	const double ground_velocity = movement_parameters["ground_velocity"];
	const double air_velocity = movement_parameters["air_velocity"];

	const double transition_time = TIME_CORRECTION_FACTOR * movement_parameters["ground_angle"] / movement_parameters["ground_velocity"] + movement_parameters["wait_time"];

	clhr->setLegPosition(tripod_1, landing_pos, air_velocity);
	clhr->setLegPosition(tripod_2, take_off_pos, ground_velocity);
	clhr->sendCommands();

	const ros::Time command_time = ros::Time::now(); 

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 3){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		loop_rate.sleep();

		time_in_state = (ros::Time::now() - command_time).toSec();
		
		if(time_in_state >= transition_time){
			clhr->transition(4);	
		}

	}

	return;
}

//Estado 3
void state_4 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Alternating tripod]: Entered state 4");

	std::vector<float> state;
	int legs_in_position = 0;

	double time_in_state = 0.0;

	update_movement_parameters(clhr->getArgs());
	const double stand_up_vel = movement_parameters["stand_up_velocity"];
	const double landing_pos = 2*PI - movement_parameters["ground_angle"]/2;
	const double take_off_pos = movement_parameters["ground_angle"]/2;
	const double ground_velocity = movement_parameters["ground_velocity"];
	const double air_velocity = movement_parameters["air_velocity"];

	const double transition_time = TIME_CORRECTION_FACTOR * movement_parameters["ground_angle"] / movement_parameters["ground_velocity"] + movement_parameters["wait_time"];

	clhr->setLegPosition(tripod_1, take_off_pos, ground_velocity);
	clhr->setLegPosition(tripod_2, landing_pos, air_velocity);
	clhr->sendCommands();

	const ros::Time command_time = ros::Time::now(); 

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 4){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		loop_rate.sleep();

		time_in_state = (ros::Time::now() - command_time).toSec();
		
		if(time_in_state >= transition_time){
			clhr->transition(3);	
		}
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

	//Sets the buffer option for the msgs
	clhr.bufferCommands(true);

	//Set the default movement parameters
	set_default_movement_parameters();

	//----------------------------------------------------
	//    State's instantiation
	//----------------------------------------------------

	//attach the states set
	clhr.attachState(1, stand_up_2_state, STARTING_STATE);
	clhr.attachState(2, state_2);
	clhr.attachState(3, state_3);
	clhr.attachState(4, state_4);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}
