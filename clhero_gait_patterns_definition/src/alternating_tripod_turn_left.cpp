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

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 100
#define PATTERN_NAME "turn_left_tripod"
#define PI 3.14159265359
#define GROUND_ANGLE (0.3490658503988659) // 20 [º]
#define AIR_ANGLE (2*PI-GROUND_ANGLE)
#define GROUND_VELOCITY (3/3) // aprox 30 [rpm]
#define AIR_VELOCITY (AIR_ANGLE/GROUND_ANGLE*GROUND_VELOCITY)
#define LEG_NUMBER 6
#define ANG_THR 0.12217304763960307
#define ANG_L2 (2*PI-GROUND_ANGLE/2.0)
#define ANG_L1 (GROUND_ANGLE/2.0)
#define STAND_UP_VEL 3
#define REV_ANG_THR PI // 300 [º]

//----------------------------------------------------
//    Global Variables
//----------------------------------------------------

//Tripods
const std::vector<int> tripod_1 = {1, 4, 5};
const std::vector<int> tripod_2 = {2, 3, 6};
const std::vector<int> all_legs = {1, 2, 3, 4, 5, 6};

//----------------------------------------------------
//    Functions
//----------------------------------------------------


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

	ROS_INFO("[Alternating tripod turn left]: Entered state 1");

	std::vector<float> state;
	int legs_in_position = 0;

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
		clhr->setLegPosition(all_legs, 0, STAND_UP_VEL);
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
				clhr->setLegPosition(i+1, 0, STAND_UP_VEL); 			
			}else{
				clhr->setLegPosition(i+1, 0, (-0.8)*STAND_UP_VEL);
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
//sets the second tripod into an air state
void state_2 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[Alternating tripod turn left]: Entered state 2");

	std::vector<float> state;
	int legs_in_position = 0;

	//clhr->setLegPosition(tripod_1, 0, GROUND_VELOCITY);
	clhr->setLegPosition(tripod_2, PI, (2)*STAND_UP_VEL);

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
			if((fabs(state[tripod_1[i]-1] - 0) < ANG_THR) || (fabs(state[tripod_1[i]-1] - (2*PI)) < ANG_THR)){
				legs_in_position++;
			}
		}

		for(int i=0; i < tripod_2.size(); i++){
			if((fabs(state[tripod_2[i]-1] - (PI)) < ANG_THR)){				
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

	ROS_INFO("[Alternating tripod turn left]: Entered state 3");

	std::vector<float> state;
	int legs_in_position = 0;

	for(int i=0; i < tripod_1.size(); i++){
		if((tripod_1[i]%2) > 0){
			//If the leg is odd (left side leg)
			clhr->setLegPosition(tripod_1[i], ANG_L2, (-1.0)*GROUND_VELOCITY);
		}else{
			//If the leg is even (right side leg)
			clhr->setLegPosition(tripod_1[i], ANG_L1, GROUND_VELOCITY);
		}
	}

	for(int i=0; i < tripod_2.size(); i++){
		if((tripod_2[i]%2) > 0){
			//If the leg is odd (left side leg)
			clhr->setLegPosition(tripod_2[i], ANG_L1, (-1.0)*AIR_VELOCITY);
		}else{
			//If the leg is even (right side leg)
			clhr->setLegPosition(tripod_2[i], ANG_L2, AIR_VELOCITY);
		}
	}

	clhr->sendCommands();

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

		state = clhr->getLegsPosition();

		for(int i=0; i < tripod_1.size(); i++){
			if((tripod_1[i]%2) > 0){
				//If the leg is odd (left side leg)
				if((fabs(state[tripod_1[i]-1] - ANG_L2) < ANG_THR)){
					legs_in_position++;
				}
			}else{
				//If the leg is even (right side leg)
				if((fabs(state[tripod_1[i]-1] - ANG_L1) < ANG_THR)){
					legs_in_position++;
				}
			}
		}

		for(int i=0; i < tripod_2.size(); i++){
			if((tripod_2[i]%2) > 0){
				//If the leg is odd (left side leg)
				if((fabs(state[tripod_2[i]-1] - ANG_L1) < ANG_THR)){
					legs_in_position++;
				}
			}else{
				//If the leg is even (right side leg)
				if((fabs(state[tripod_2[i]-1] - ANG_L2) < ANG_THR)){
					legs_in_position++;
				}
			}
		}

		if(legs_in_position == LEG_NUMBER){
			clhr->transition(4);
		}else{
			legs_in_position = 0;
		}

		loop_rate.sleep();
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

	ROS_INFO("[Alternating tripod turn left]: Entered state 4");

	std::vector<float> state;
	int legs_in_position = 0;

	for(int i=0; i < tripod_1.size(); i++){
		if((tripod_1[i]%2) > 0){
			//If the leg is odd (left side leg)
			clhr->setLegPosition(tripod_1[i], ANG_L1, (-1.0)*AIR_VELOCITY);
		}else{
			//If the leg is even (right side leg)
			clhr->setLegPosition(tripod_1[i], ANG_L2, AIR_VELOCITY);
		}
	}

	for(int i=0; i < tripod_2.size(); i++){
		if((tripod_2[i]%2) > 0){
			//If the leg is odd (left side leg)
			clhr->setLegPosition(tripod_2[i], ANG_L2, (-1.0)*GROUND_VELOCITY);
		}else{
			//If the leg is even (right side leg)
			clhr->setLegPosition(tripod_2[i], ANG_L1, GROUND_VELOCITY);
		}
	}

	clhr->sendCommands();

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

		state = clhr->getLegsPosition();

		for(int i=0; i < tripod_1.size(); i++){
			if((tripod_1[i]%2) > 0){
				//If the leg is odd (left side leg)
				if((fabs(state[tripod_1[i]-1] - ANG_L1) < ANG_THR)){
					legs_in_position++;
				}
			}else{
				//If the leg is even (right side leg)
				if((fabs(state[tripod_1[i]-1] - ANG_L2) < ANG_THR)){
					legs_in_position++;
				}
			}
		}

		for(int i=0; i < tripod_2.size(); i++){
			if((tripod_2[i]%2) > 0){
				//If the leg is odd (left side leg)
				if((fabs(state[tripod_2[i]-1] - ANG_L2) < ANG_THR)){
					legs_in_position++;
				}
			}else{
				//If the leg is even (right side leg)
				if((fabs(state[tripod_2[i]-1] - ANG_L1) < ANG_THR)){
					legs_in_position++;
				}
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
