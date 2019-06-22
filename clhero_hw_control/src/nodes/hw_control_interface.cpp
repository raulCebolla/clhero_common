//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File:
//	Version:
//	Description:
//	Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <clhero_gait_controller/LegCommand.h>
#include <clhero_gait_controller/LegState.h>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>
#include <thread>
#include <epos_functions/epos_functions.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define LOOP_RATE 100 //Rate at which the node checks for callbacks
#define LEG_NUMBER 6 //Number of legs of the robot
#define CONTROL_RATE 100 //Rate which the node sends the control of each leg
#define PI 3.14159265359
#define ANG_THR 0.03490658503988659 //2*(2*PI)/360
#define ANG_V 0.17453292519943295 //10*(2*PI)/360
#define STATE_UPDATE_RATE 200 //Rate at which the state of the legs is update [Hz]
#define POS_COMMAND_THR 0.008726646259971648 //Threshold in which a position command is considered the same [0.5 deg]

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Publisher of the legs_state_msg
ros::Publisher legs_state_pub;

//Publisher of each of the command msgs
std::vector<ros::Publisher> controller_command_pub;

//EPOS control functions class instantiation
epos_functions* epos;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that turns rad/s into rpm
double rads2rpm (double rads){
	return rads*60/(2*PI);
}

//Function that turns a relative position command [0, 2pi) into an absolute position based
//on the accumulate position of the motor
double turnAbsolutePosition (double pos_command, double vel_command, double curr_position){
	
	long n = 0;
	double fixed_command;

	//Gets the number of turns
	n = trunc(curr_position/(2*PI));

	//Calcs the absolute position depending on the sign of the turn's number
	if (n >= 0){
		fixed_command = 2*PI*n + pos_command;
	}else{
		fixed_command = 2*PI*n - (2*PI - pos_command); 
	}

	//Depending on the movement's direction, the final position shall be corrected 
	if(vel_command > 0){
		//If the final position is lower than the current position on forward movement
		if((fixed_command < curr_position) && (fabs(fixed_command - curr_position) > POS_COMMAND_THR)){
			fixed_command += 2*PI;
		}
	}else{
		//If the final position is greater than the current position on backward movement
		if((fixed_command > curr_position) && (fabs(fixed_command - curr_position) > POS_COMMAND_THR)){
			fixed_command -= 2*PI;
		}
	}

	return fixed_command;
}

//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  return (angle - 2*PI*trunc(angle/(2*PI)));
}

//Function that parses the incoming leg command msg
clhero_gait_controller::LegCommand parseLegCommand (const clhero_gait_controller::LegCommand::ConstPtr& msg){
	//Declares the parsed command msg
	clhero_gait_controller::LegCommand command;

	//For each leg
	for(int i=0; i<LEG_NUMBER; i++){
		//Depending on the velocity's sign
		if(msg->vel[i] > 0){
			//Velocity is positive

			//
		}else{

		}
	}
}

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

  //Repeats for each of the legs
  for(int i=0; i<LEG_NUMBER; i++){
  	
  	//First checks the mode: position or velocity
		if(msg->position_command[i].data){
			//Position command
			epos->ActivateProfilePosition(i+1);
			//Checks if a new profile shall be set
			if(msg->new_acel_profile[i].data){
				//If so, uploads the acceleration and decceleration
				epos->SetPositionProfile(i+1, (int)rint(msg->vel[i]), (int)rint(msg->acel[i]), (int)rint(msg->decel[i]));
			}

			//Once the position has been set sends the order
			epos->MoveToPosition(i+1, (int)rint(msg->pos[i]), true, true);
		}else{
			//Velocity command
			epos->ActivateProfileVelocity(i+1);
			//Checks if a new profile shall be set
			if(msg->new_acel_profile[i].data){
				//If so, uploads the acceleration and decceleration
				epos->SetVelocityProfile(i+1, (int)rint(msg->acel[i]), (int)rint(msg->decel[i]));
			}

			//Once the position has been set sends the order
			epos->MoveWithVelocity(i+1, (int)rint(msg->vel[i]));
		}
  }

  return;
}

//Thread that periodically updates the state of the legs
void StateUpdateThread (){

	//Leg State msg
	clhero_gait_controller::LegState leg_state_msg;

	//Creates the ros rate
	ros::Rate state_update_rate (STATE_UPDATE_RATE);

	//Core loop of the thread
	while(ros::ok()){

		//For each leg
		for(int i = 0; i < LEG_NUMBER; i++){
			leg_state_msg.pos.push_back(epos->GetPosition(i+1));
    			leg_state_msg.vel.push_back(epos->GetVelocity(i+1));
    			leg_state_msg.torq.push_back(0);
		}

		//Publishes the msg
		legs_state_pub.publish(leg_state_msg);

		leg_state_msg.pos.clear();
		leg_state_msg.vel.clear();
		leg_state_msg.torq.clear();

		//Sleeps for each loop
		state_update_rate.sleep();
	}

	return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_simulation_interface");
  ros::NodeHandle nh;

  ros::Rate loop_rate (LOOP_RATE);

  //Creates the maxon motors'handler
  epos = new epos_functions();

  //Publishers
  legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 1000);
  
  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 1000, legCommandCallback);
  //ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1000, jointStatesCallback);
  
  //threads which helds the status publishing function
  std::thread state_update_thr (StateUpdateThread);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------

  ros::spin();

  //----------------------------------------------------
  //    End of node statements
  //----------------------------------------------------

  state_update_thr.join();

  delete epos;

  return 0;

}
