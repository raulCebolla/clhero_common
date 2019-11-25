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
#include <clhero_gait_controller/OffsetSetting.h>
#include <clhero_hw_control/RawState.h>
#include <clhero_hw_control/GetErrors.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <epos_functions/epos_functions.h>

#include <ros/callback_queue.h>
//#include <chrono_register/chrono_register.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define LOOP_RATE 500 //Rate at which the node checks for callbacks
#define LEG_NUMBER 6 //Number of legs of the robot
#define CONTROL_RATE 100 //Rate which the node sends the control of each leg
#define PI 3.14159265359
#define ANG_THR 0.03490658503988659 //2*(2*PI)/360
#define ANG_V 0.17453292519943295 //10*(2*PI)/360
#define STATE_UPDATE_RATE 200 //Rate at which the state of the legs is update [Hz]
#define POS_COMMAND_THR 0.08726646259971647 //Threshold in which a position command is considered the same [0.5 deg]
#define DEFAULT_VEL (2*3.141592653589793) // 0.5 rev/s = 30 rpm
#define DEFAULT_ACCEL 1256.6370614359172 // 1200 rpm/s
#define DEFAULT_DECEL 1256.6370614359172 // 1200 rpm/s
#define ERROR_THR 1e-6

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

class CommandMsgManager{

	clhero_gait_controller::LegCommand current_command;
	clhero_gait_controller::LegCommand fixed_command;
	epos_functions* epos;
	std::vector<bool> is_new_command;

	//Function that checks if one leg has its turning direction with the opposite sign
	bool checkNegativeMotor (int motor);

	void fixNewCommand ();

public:

	CommandMsgManager(epos_functions* e);

	void evaluateNewCommand(const clhero_gait_controller::LegCommand::ConstPtr& msg);

	void commandAllMotors();

};

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Publisher of the legs_state_msg
ros::Publisher legs_state_pub;
ros::Publisher raw_state_pub;

//State publisher for tf 
ros::Publisher joint_state_pub;

//Publisher of each of the command msgs
std::vector<ros::Publisher> controller_command_pub;

//EPOS control functions class instantiation
epos_functions* epos_f;

//Command msg manager
CommandMsgManager* com_man;

//Register manager 
//ChronoRegister *reg, *s_reg;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that maps the motors
int mapMotor (int motor){
	switch (motor){
		case 1:
			return 6;
		case 2:
			return 5;
		case 3:
			return 4;
		case 4:
			return 3;
		case 5:
			return 2;
		case 6:
			return 1;
		default:
			return 0;
	}
}

//Function that turns rad/s into rpm
inline double rads2rpm (double rads){
	return rads*60/(2*PI);
}

//Function that checks if one leg has its turning direction with the opposite sign
inline bool checkNegativeMotor (int motor){
	switch (motor){
		case 1:
			return true;
		case 3:
			return true;
		case 5:
			return true;
		default:
			return false;
	}
	return false;
}

//Function that turns a relative position command [0, 2pi) into an absolute position based
//on the accumulate position of the motor
double turnAbsolutePosition (double pos_command, double vel_command, double curr_position){

	//Stopwatch watch;

	long n = 0;
	double fixed_command, diff, rel_curr_pos;

	//watch.start();
	//Gets the number of turns
	n = floor(curr_position/(2*PI));

	//Calcs the relative current position of the leg, its position between [0,360]
	rel_curr_pos = curr_position - 2*PI*n;

	//Depending on the movement's direction, the final position shall be corrected 
	if(vel_command > 0){

		diff = pos_command - rel_curr_pos;
		if (diff < 0){
			diff += 2*PI;
		}
		fixed_command = curr_position + diff;

	}else{

		diff = rel_curr_pos - pos_command;
		if (diff < 0){
			diff += 2*PI;
		}
		fixed_command = curr_position - diff;

	}
	
	//watch.stop();
	//reg->write("turnAbsolutePosition", watch.get_interval());

	return fixed_command;

}

//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  return (angle - 2*PI*trunc(angle/(2*PI)));
}

void StateUpdateMethod (){

	//Leg State msg
	clhero_gait_controller::LegState leg_state_msg;
	//clhero_hw_control::RawState raw_state_msg;

	//Joint state msgs
	sensor_msgs::JointState joint_state_msg;
	std::string joint_name;

	double position, velocity, effort;
	int n = 0;

	//std::vector<double> readings_reg;	

	leg_state_msg.stamp = ros::Time::now();
	//raw_state_msg.stamp = leg_state_msg.stamp;	
	joint_state_msg.header.stamp = leg_state_msg.stamp;

	//For each leg
	for(int i = 0; i < LEG_NUMBER; i++){
		
		position = epos_f->GetPosition(mapMotor(i+1));
		velocity = epos_f->GetVelocity(mapMotor(i+1));
		effort = epos_f->GetEffort(mapMotor(i+1));
		n = trunc(position/(2*PI));
		position -= 2*PI*n;
		
		if (position < 0){
			position += 2*PI;
		}

		if(checkNegativeMotor(mapMotor(i+1))){
			position = 2*PI - position;
			velocity = (-1.0)*velocity;
			effort = (-1.0)*effort;
		}
	
		leg_state_msg.pos.push_back(position);
		leg_state_msg.vel.push_back(velocity);
		leg_state_msg.torq.push_back(effort);

		/*
		raw_state_msg.pos.push_back(epos_f->GetRawPosition(mapMotor(i+1)));
		raw_state_msg.vel.push_back(epos_f->GetRawVelocity(mapMotor(i+1)));
		raw_state_msg.eff.push_back(epos_f->GetRawEffort(mapMotor(i+1)));
		*/

		//For legs state
		joint_name = "motor_" + std::to_string(i+1) + "_to_base_link_joint";

		joint_state_msg.name.push_back(joint_name);
		joint_state_msg.position.push_back(position);
		joint_state_msg.velocity.push_back(velocity);
		joint_state_msg.effort.push_back(effort);
		
		/*
		readings_reg.push_back((double)i);
		readings_reg.push_back(position);
		readings_reg.push_back(velocity);
		readings_reg.push_back(effort);
		*/
		//s_reg->write(readings_reg);
		//readings_reg.clear();
	}

	//Publishes the msg
	legs_state_pub.publish(leg_state_msg);
	//raw_state_pub.publish(raw_state_msg);
	joint_state_pub.publish(joint_state_msg);

	return;
}


//----------------------------------------------------
//    Callbacks
//----------------------------------------------------

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

	//Stopwatch watch;

	//watch.start();
	com_man->evaluateNewCommand(msg);
	com_man->commandAllMotors();
	//watch.stop();

	//reg->write("legCommandCallback", watch.get_interval());

 	return;
}

//Callback for setting the position offset
void offsetSettingCallback(const clhero_gait_controller::OffsetSetting::ConstPtr &msg){
	
	double actual_pos;

	for(int i = 0; i < msg->id.size(); i++){
		
		if(checkNegativeMotor(mapMotor(msg->id[i]))){
			actual_pos = 2*PI - msg->actual_pos[i];
		}else{
			actual_pos = msg->actual_pos[i];
		}

		epos_f->setPositionOffset(mapMotor(msg->id[i]), actual_pos);
	
	}

	return;
}

bool get_errors_srv_callback (clhero_hw_control::GetErrors::Request &req,
              				  clhero_hw_control::GetErrors::Response &res){

  for (int i = 1; i<=LEG_NUMBER; i++){
  	epos_f->GetErrors(i);
  }

  res.ok = true;

  return true;

}

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------


//Function that checks if one leg has its turning direction with the opposite sign
bool CommandMsgManager::checkNegativeMotor (int motor){

	switch (motor){
		case 1:
			return true;
		case 3:
			return true;
		case 5:
			return true;
		default:
			return false;
	}
	return false;
}

void CommandMsgManager::fixNewCommand (){

	//Stopwatch watch, activate_profile_watch, set_profile_watch;

	//watch.start();
	//For each leg
	for(int i=0; i<LEG_NUMBER; i++){
		//If there is a new command to be treated
		if(is_new_command[i]){
			//Command for this leg
			fixed_command.pos[i] = current_command.pos[i];
			fixed_command.vel[i] = current_command. vel[i];
			fixed_command.acel[i] = current_command.acel[i];
			fixed_command.decel[i] = current_command.decel[i];	
			fixed_command.new_acel_profile[i].data = current_command.new_acel_profile[i].data;
			fixed_command.position_command[i].data = current_command.position_command[i].data;

			//First checks the mode: position or velocity
			if(current_command.position_command[i].data){
				//Position command

				//Absolute position
				double abs_position;

				//Activates the position mode
				//activate_profile_watch.start();
				this->epos->ActivateProfilePosition(mapMotor(i+1));
				//activate_profile_watch.stop();
				//reg->write("epos::ActivateProfilePosition", activate_profile_watch.get_interval());
				//Corrects the command if the motor is negative-turn
				if(this->checkNegativeMotor(mapMotor(i+1))){
					fixed_command.pos[i] = 2*PI - fixed_command.pos[i];
					fixed_command.vel[i] = (-1.0)*fixed_command.vel[i]; 
				}
				//Checks if a new profile shall be set
				if(current_command.new_acel_profile[i].data){
					//If so, uploads the acceleration and decceleration
					//set_profile_watch.start();
					epos->SetPositionProfile(mapMotor(i+1), fabs(fixed_command.vel[i]), fixed_command.acel[i], fixed_command.decel[i]);
					//set_profile_watch.stop();
					//reg->write("epos::SetPositionProfile", set_profile_watch.get_interval());
				}

				//Gets the absolute position based on the position command, velocity and current position
				fixed_command.pos[i] = turnAbsolutePosition(fixed_command.pos[i], fixed_command.vel[i], this->epos->GetPosition(mapMotor(i+1)));
			}else{
				//Velocity command

				//Activates the velocity profile
				//activate_profile_watch.start();
				this->epos->ActivateProfileVelocity(mapMotor(i+1));
				//activate_profile_watch.stop();
				//reg->write("ActivateProfileVelocity", activate_profile_watch.get_interval());
				//Corrects the command if the motor is negative-turn
				if(this->checkNegativeMotor(mapMotor(i+1))){
					fixed_command.pos[i] = 2*PI - fixed_command.pos[i];
					fixed_command.vel[i] = (-1.0)*fixed_command.vel[i];
				}
				//Checks if a new profile shall be set
				if(current_command.new_acel_profile[i].data){
					//If so, uploads the acceleration and decceleration
					//set_profile_watch.start();
					this->epos->SetVelocityProfile(mapMotor(i+1), fixed_command.acel[i], fixed_command.decel[i]);
					//set_profile_watch.stop();
					//reg->write("SetVelocityProfile", set_profile_watch.get_interval());
				}
			}
		}
	}
	//watch.stop();

	//reg->write("CommandMsgManager::fixNewCommand", watch.get_interval());

	return;
}

CommandMsgManager::CommandMsgManager(epos_functions* e){
	
	this->epos = e;

	for(int i=0; i<LEG_NUMBER; i++){

		is_new_command.push_back(false);
		
		current_command.pos[i] = -1003;
		current_command.vel[i] = current_command.acel[i] = current_command.decel[i] = -10e6;
		current_command.new_acel_profile[i].data = false;
		current_command.position_command[i].data = false;

		fixed_command.pos[i] = -1003;
		fixed_command.vel[i] = fixed_command.acel[i] = fixed_command.decel[i] = -10e6;
		fixed_command.new_acel_profile[i].data = false;
		fixed_command.position_command[i].data = false;
	}

}

void CommandMsgManager::evaluateNewCommand(const clhero_gait_controller::LegCommand::ConstPtr& msg){
	
	//Stopwatch watch;

	//watch.start();
	for(int i=0; i<LEG_NUMBER; i++){

		if(fabs(current_command.pos[i] - msg->pos[i]) > ERROR_THR){
			
			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.vel[i] - msg->vel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.acel[i] - msg->acel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.decel[i] - msg->decel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(current_command.new_acel_profile[i].data != msg->new_acel_profile[i].data){
			
			is_new_command[i] = true;
			continue;

		}else if(current_command.position_command[i].data != msg->position_command[i].data){
			
			is_new_command[i] = true;
			continue;
							
		}
	}

	for(int i=0; i<LEG_NUMBER; i++){
		if(is_new_command[i]){
			current_command.pos[i] = msg->pos[i];
			current_command.vel[i] = msg->vel[i];
			current_command.acel[i] = msg->acel[i];
			current_command.decel[i] = msg->decel[i];
			current_command.new_acel_profile[i].data = msg->new_acel_profile[i].data;
			current_command.position_command[i].data = msg->position_command[i].data;			
		}
	}
	//watch.stop();

	//reg->write("CommandMsgManager::evaluateNewCommand", watch.get_interval());

	this->fixNewCommand();
	return;
}

void CommandMsgManager::commandAllMotors(){

	//Stopwatch watch, move_to_pos_watch, move_w_vel_watch;

	//watch.start();
	for(int i=0; i<LEG_NUMBER; i++){
		if(is_new_command[i]){
			if(fixed_command.position_command[i].data){
				//Position command
				//move_to_pos_watch.start();
				this->epos->MoveToPosition(mapMotor(i+1), fixed_command.pos[i], true, true);
				//move_to_pos_watch.stop();
				//reg->write("epos::MoveToPosition", move_to_pos_watch.get_interval());
			}else{
				//Velocity command
				//move_w_vel_watch.start();
				this->epos->MoveWithVelocity(mapMotor(i+1), fixed_command.vel[i]);
				//move_w_vel_watch.stop();
				//reg->write("epos::MoveWithVelocity", move_w_vel_watch.get_interval());
			}
		}
		is_new_command[i] = false;
	}
	//watch.stop();

	//reg->write("CommandMsgManager::commandAllMotors", watch.get_interval());

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

  //Register initialization
  //reg = new ChronoRegister;
  //reg->set_path("/home/hexapodo/chrono_register_results");
  //s_reg = new ChronoRegister;
  //s_reg->set_path("/home/hexapodo/clhero_state_results");  

  //Stopwatch loop_watch, state_watch, callback_watch;

  //Creates the maxon motors'handler
  epos_f = new epos_functions();

  //Sets the default profile
  for(int i = 0; i < LEG_NUMBER; i++){
  	epos_f->ActivateProfilePosition(mapMotor(i+1));
  	epos_f->SetPositionProfile(mapMotor(i+1), DEFAULT_VEL, DEFAULT_ACCEL, DEFAULT_DECEL);
  	epos_f->SetVelocityProfile(mapMotor(i+1), DEFAULT_ACCEL, DEFAULT_DECEL);
  }

  //Creates the msg manager
  com_man = new CommandMsgManager(epos_f);

  //Publishers
  legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 10000);
  raw_state_pub = nh.advertise<clhero_hw_control::RawState>("legs_raw_state", 10000);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10000);
  
  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 10000, legCommandCallback);
  ros::Subscriber offset_sub = nh.subscribe("offset_setting", 10000, offsetSettingCallback);

  ros::ServiceServer get_errors_service = nh.advertiseService("get_motor_errors", get_errors_srv_callback);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------
  
  //ros::waitForShutdown();

  while(ros::ok()){
  	//loop_watch.start();
  	
 	//state_watch.start();
  	StateUpdateMethod();
  	//state_watch.stop();

  	//callback_watch.start();
  	ros::spinOnce();
  	//callback_watch.stop();

  	//loop_watch.stop();

  	//reg->write("Loop:", loop_watch.get_interval());
  	//reg->write("callback:", callback_watch.get_interval());
  	//reg->write("state:", state_watch.get_interval());
  	//loop_rate.sleep();
  }

  epos_f->closeAllDevices();

  //----------------------------------------------------
  //    End of node statements
  //----------------------------------------------------

  //state_update_thr.join();

  delete epos_f;
  delete com_man;

  return 0;

}
