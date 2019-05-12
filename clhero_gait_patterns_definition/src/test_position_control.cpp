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
#include <iostream>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 50
#define PATTERN_NAME "position_control_test"
#define PI 3.1416

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Declare the functions needed in the gait pattern

//----------------------------------------------------
//    States
//----------------------------------------------------

//Estado 1
void state_1 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(1.0/5);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//As for example
	for(int i=0; i<6; i++){
		clhr->setLegPosition(i+1, 90*2*PI/(360), 3);
	}

	std::cout << "[position_control_test]: Entered state 1" << std::endl;

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

		clhr->transition(2);
	}

	return;
}

//Estado 2
void state_2 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(1.0/5);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//As for example
	for(int i=0; i<6; i++){
		clhr->setLegPosition(i+1, 180*2*PI/(360), 3);
	}

	std::cout << "[position_control_test]: Entered state 2" << std::endl;

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

		loop_rate.sleep();

		clhr->transition(3);
	}

	return;
}

//Estado 1
void state_3 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(1.0/5);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//As for example
	for(int i=0; i<6; i++){
		clhr->setLegPosition(i+1, 270*2*PI/(360), 3);
	}

	std::cout << "[position_control_test]: Entered state 3" << std::endl;

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

		clhr->transition(1);
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

	//----------------------------------------------------
	//    State's instantiation
	//----------------------------------------------------

	//attach the states set
	clhr.attachState(1, state_1, STARTING_STATE);
	clhr.attachState(2, state_2);
	clhr.attachState(3, state_3);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}