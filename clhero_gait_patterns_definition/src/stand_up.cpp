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

#define STATE_LOOP_RATE 200
#define PATTERN_NAME "stand_up"
#define PI 3.14159265359
#define GROUND_ANGLE (1.5707963267948966) // 60 [º]
#define AIR_ANGLE (2*PI-GROUND_ANGLE)
#define GROUND_VELOCITY 3 // aprox 30 [rpm]
#define DEFAULT_VEL 2 //[rad/s]
#define AIR_VELOCITY (AIR_ANGLE/GROUND_ANGLE*GROUND_VELOCITY)
#define LEG_NUMBER 6
#define ANG_THR 0.12217304763960307
#define LANDING_ANG (2*PI-GROUND_ANGLE/2.0)
#define TAKE_OFF_ANG (GROUND_ANGLE/2.0)
#define REV_ANG_THR 5.235987755982989 // 300 [º]
#define STAND_UP_VEL 2

//----------------------------------------------------
//    Global Variables
//----------------------------------------------------

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

	ROS_INFO("[Stand up]: Entered state 1");

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
		clhr->setLegPosition(all_legs, 0, DEFAULT_VEL);
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
				clhr->setLegPosition(i+1, 0, (-0.8)*STAND_UP_VEL); 			
			}else{
				clhr->setLegPosition(i+1, 0, STAND_UP_VEL);
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
	clhr.attachState(1, stand_up_2_state, STARTING_STATE);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}
