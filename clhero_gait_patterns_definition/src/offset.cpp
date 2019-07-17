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
#include <clhero_gait_controller/OffsetSetting.h>
#include <string>
#include <vector>
#include <cmath>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 200
#define PATTERN_NAME "offset_setting"
#define TEST_VEL 0.25
#define EFF_THR 500
#define RECOVER_VEL 3
#define REST_ANG 3.490658503988659 //[rads] = 200[º]
#define MAX_TAKE_OFF_ANG 1.8104073438134423 //[rads] = 103.7287 [º]

#ifndef LEG_NUMBER
#define LEG_NUMBER 6
#endif

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Declare the functions needed in the gait pattern

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

ros::Publisher offset_pub;

//----------------------------------------------------
//    States
//----------------------------------------------------

//Estado 1
void state_1 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//Flags indicating which of the legs have already been set the offset
	std::vector<bool> is_offset_set (LEG_NUMBER, false);

	//Vector with the current effort on each leg
	std::vector<float> effort;

	//Flags to indicate if an offset msg shall be sent
	bool is_new_msg = false;

	//Number of legs in position
	unsigned int legs_set = 0;

	//Offset setting msg
	clhero_gait_controller::OffsetSetting offset_msg;

	//Set backwards movement on each leg
	for(int i = 0; i < LEG_NUMBER; i++){
		clhr->setLegVelocity(i+1, (-1.0)*TEST_VEL);
	}

	clhr->sendCommands();

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 1){

		//Reads the efforts
		effort = clhr->getLegsEffort();		

		//For each of the legs
		for(int i=0; i < LEG_NUMBER; i++){

			//If this leg has not set the offset yet
			if(!is_offset_set[i]){
				//If the effort surpases the effort threshold
				if(effort[i] < (-1.0)*EFF_THR){
					ROS_INFO("Leg %d in position");
					//Halts the movement
					clhr->setLegVelocity(i+1, 0);
					clhr->sendCommands();
					//Prepares the msg
					offset_msg.id.push_back(i+1);
					offset_msg.actual_pos.push_back(MAX_TAKE_OFF_ANG);
					//Mark this leg as set
					is_offset_set[i] = true;
					//Mark that the msg shall be sent
					is_new_msg = true;
				}
			}
		}

		//If a msg shall be sent
		if(is_new_msg){
			//Sends the msg
			offset_pub.publish(offset_msg);
			is_new_msg = false;

			//Clears the msg
			offset_msg.id.clear();
			offset_msg.actual_pos.clear();
		}

		//Counts the number of legs set
		for(int i=0; i<is_offset_set.size(); i++){
			if(is_offset_set[i]){
				legs_set++;
			}
		}

		//If all the legs are set, transition to the next state
		if(legs_set >= LEG_NUMBER){
			ROS_INFO("[Offset_setting] All offsets configured.");
			clhr->transition(2);
		}else{
			legs_set = 0;
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

	ros::Rate loop_rate(100);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//As for example
	for(int i=0; i<LEG_NUMBER; i++){
		clhr->setLegPosition(i+1, REST_ANG, RECOVER_VEL);
	}

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

	//Advertise the offset publisher
	offset_pub = nh.advertise<clhero_gait_controller::OffsetSetting>("offset_setting", 10);

	//----------------------------------------------------
	//    State's instantiation
	//----------------------------------------------------

	//attach the states set
	clhr.attachState(1, state_1, STARTING_STATE);
	clhr.attachState(2, state_2);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}
