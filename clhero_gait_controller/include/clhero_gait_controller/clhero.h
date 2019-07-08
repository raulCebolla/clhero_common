
//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File: clhero.h
//	Version: 1.0
//	Description: Header for clhero library
//	Changelog:
//=====================================================================

#ifndef CLHERO_H
#define CLHERO_H

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <vector>
#include <clhero_gait_controller/LegCommandRequest.h>
#include <clhero_gait_controller/PatternCommand.h>
#include <clhero_gait_controller/LegState.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <map>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define DEFAULT_VEL (2*3.141592653589793) // pi [rad/s]
#define DEFAULT_ACEL 1256.6370614359172
#define DEFAULT_GP_NAME "None"
#define STARTING_STATE true

namespace clhero{

//----------------------------------------------------
//    Type declaration
//----------------------------------------------------

//Enumeration that specify the status in which the robot is
enum robot_status {stop, pause, active, test};

//Struct containing the state of the legs
struct LegState{
	std::vector<float> pos;
	std::vector<float> vel;
	std::vector<float> torq;
};

//----------------------------------------------------
//    Class declaration
//----------------------------------------------------

//The clhero class represents an clhrapod robot for command purposes. This class provides 
//a common methodoly for command and gait pattern implementation
class Clhero_robot {

	//----------------------------------------------------
	//    ROS fields 
	//----------------------------------------------------

	ros::NodeHandle nh;

	//Subscribe client for the command service
	ros::ServiceClient command_srv_client;

	//Robot status
	robot_status status;

	//----------------------------------------------------
	//    Fields 
	//----------------------------------------------------

	//ID of each of the class' instances
	int id;

	//Field containing the gait pattern name
	std::string gp_name;

	//Fields containing the arguments of the movement
	//std::vector<float> args; 
	std::map<std::string, std::string> args;

	//Thread that handles the pattern command msg reception
	std::thread *pattern_command_reception_thread;

	//Thread that handles the pattern command msg reception
	std::thread *leg_state_reception_thread;

	//Thread that handles the state functions
	std::thread *state_handler_thread;

	//Thread with the active state
	std::thread *active_state_thread;

	//Hash map to contain the attached functions that belongs to each of the different
	//declared states
	std::unordered_map<int, void(*)(Clhero_robot*)> state_fcn_map;

	//Active state
	int active_state;

	//Starting state
	int starting_state;

	//Leg's Status considered: position, velocity and torque
	LegState leg_state;

	//----------------------------------------------------
	//    Private methods 
	//----------------------------------------------------

	//Friend function that handles the reception of the pattern command msg
	friend void handle_pattern_command_msg(const clhero_gait_controller::PatternCommand::ConstPtr& msg);

	//Friend function used to handle the reception of the pattern command as a separate
	//thread
	friend void handle_pattern_command_reception (Clhero_robot* clhr);

	//Friend function that handles the reception of the leg status msg
	friend void handle_leg_state_msg(const clhero_gait_controller::LegState::ConstPtr& msg);

	//Friend function used to handle the reception of the leg status as a separate
	//thread
	friend void handle_leg_state_reception (Clhero_robot* clhr);

	//Friend function used to handle the active state threads
	friend void handle_active_state(Clhero_robot* clhr);

	//Function that forces a transition
	bool forceTransition (int state_id);

public:

	//Default constructor
	Clhero_robot(std::string pattern_name = DEFAULT_GP_NAME);

	//Set the position for a leg
	void setLegPosition (int leg, 
						float positions, 
						float velocity = DEFAULT_VEL,
						bool new_acel_profile = true,
						float acel = DEFAULT_ACEL,
						float decel = DEFAULT_ACEL);

	//Set the Velocity for a leg
	void setLegVelocity (int leg,  
						float velocity,
						bool new_acel_profile = true,
						float acel = DEFAULT_ACEL,
						float decel = DEFAULT_ACEL);

	//Set the gait pattern name
	void setGaitPatternName (std::string name);

	//Function to attach a new state
	//Returns:
	//	True - if the state was successfully attached
	//	False - if the state couldn't be attached
	bool attachState (int state_id, void(*state_fcn)(Clhero_robot*), bool starting_state_flg = false);

	//Function that returns the active state
	int activeState ();

	//Function that returns the starting state
	int getStartingState ();

	//Function that performs the transition to a new state given as argument
	bool transition (int state_id);

	//Function that lets the gait pattern run
	void run ();

	//Function that returns the state of the legs
	LegState getLegState ();
	std::vector<float> getLegsPosition();
	std::vector<float> getLegsVelocity();
	std::vector<float> getLegsTorque();

	//Function that returns the arguments
	std::map<std::string, std::string> getArgs();

};

//----------------------------------------------------
//    Function declaration
//----------------------------------------------------

//Function used to register a new gait pattern on the parameter server
void registerGaitPattern(std::string name);

//Friend function that handles the reception of the pattern command msg
void handle_pattern_command_msg(const clhero_gait_controller::PatternCommand::ConstPtr& msg);

//Friend function used to handle the reception of the pattern command as a separate
//thread
void handle_pattern_command_reception (Clhero_robot* clhr);

//Friend function that handles the reception of the leg status msg
void handle_leg_state_msg(const clhero_gait_controller::LegState::ConstPtr& msg);

//Friend function used to handle the reception of the leg status as a separate
//thread
void handle_leg_state_reception (Clhero_robot* clhr);

//Friend function used to handle the active state threads
void handle_active_state(Clhero_robot* clhr);

}


#endif