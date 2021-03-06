
//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File: clhero.h
//	Version: 1.0
//	Description: Header for clhero library
//	Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <clhero_gait_controller/clhero.h>
#include <clhero_gait_controller/PatternCommand.h>
#include <clhero_gait_controller/RegisterGaitPattern.h>
#include <mutex>
#include <map>
#include <vector>
#include <string>
#include <cstddef>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define CURRENT_GP_PARAM_NAMESPACE "/clhero/gait_patterns/current"
#define REGISTERED_GP_PARAM_NAMESPACE "/clhero/gait_patterns/registered"
#define LEG_NUMBER 6
#define PATTERN_COMMAND_RECEPTION_RATE 100
#define LEG_STATE_RECEPTION_RATE 200
#define REGISTRATION_SRV_NOT_AVAILABLE_LOOP_RATE 50
#define REGISTRATION_SRV_NOT_AVAILABLE_TRY_LIMIT (20*REGISTRATION_SRV_NOT_AVAILABLE_LOOP_RATE)
//#define DEBUG_MODE true

//----------------------------------------------------
//    Namespace
//----------------------------------------------------

using namespace clhero;

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Pointers to each of the class'instances in a node
std::vector<clhero::Clhero_robot*> instances_ptr;

//Mutex used to sychronize the write on the state of the legs
std::mutex leg_state_mutex;

//----------------------------------------------------
//    Function definition
//----------------------------------------------------

//Function that creates the argument dictionary from a formated string
//The string shall be formated by: key: value, key: value, [...]
std::map<std::string, std::string> parseargs2map (std::string args_str){
	//Map with the mapped arguments
	std::map<std::string, std::string> arg_map;
	//Vector with the key value pairs
	std::vector<std::string> key_value_pairs;

	//First separates the input string into the different key-values pairs
	std::size_t pos = 0, prev_pos = 0;
	while((pos = args_str.find(',', pos+1)) != std::string::npos){
		if(!prev_pos){
			//if it is the first pair
			key_value_pairs.push_back(args_str.substr(prev_pos, (pos - prev_pos)));
		}else{
			key_value_pairs.push_back(args_str.substr(prev_pos + 1, (pos - prev_pos)));
		}
		prev_pos = pos;
	}
	//Adds the last to the list
	if(!prev_pos){
		//if it is the first pair
		key_value_pairs.push_back(args_str.substr(prev_pos));
	}else{
		key_value_pairs.push_back(args_str.substr(prev_pos + 1));
	}

	//Parses each key-value pair
	std::size_t separator_pos = 0;
	std::string key, value;
	for(int i=0; i<key_value_pairs.size(); i++){
		//For each pair

		if((separator_pos = key_value_pairs[i].find(':')) == std::string::npos){
			//if a pair does not have a separator mark (:), discard this pair
			continue;
		}else{
			key = key_value_pairs[i].substr(0, separator_pos);
			value = key_value_pairs[i].substr(separator_pos + 1);

			//erases the whitespaces at the beginning and in the end of the string
			if(key[0] == ' '){
				key.erase(0,1);
			}
			if(key[key.size()-1] == ' '){
				key.erase(key.size()-1,1);
			}
			if(value[0] == ' '){
				value.erase(0,1);
			}
			if(value[value.size()-1] == ' '){
				value.erase(value.size()-1,1);
			}

			//Inserts the pair to the map
			arg_map[key] = value;
		}
	}

	return arg_map;
}

//Friend function that handles the pattern command msg
void clhero::handle_pattern_command_msg(const clhero_gait_controller::PatternCommand::ConstPtr& msg){
	//Checks if the msg is for this gait pattern
	if(instances_ptr[0]->gp_name.compare(msg->pattern_name)){
		//if not for this gait pattern, return
		return;
	}

	Clhero_robot* hex = instances_ptr[0];

	//---Start order
	if(msg->order.compare("start") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Changes the status to active
		hex->status = active;

	}

	//---stop order
	if(msg->order.compare("stop") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Forces a transition to the zero state
		hex->forceTransition(0);
		//Changes the status to stopped
		hex->status = stop;

	}

	//---pause order
	if(msg->order.compare("pause") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Changes the status to paused
		hex->status = pause;

	}

	//---continue order
	if(msg->order.compare("continue") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Changes the status to active
		hex->status = active;

	}

	//---force_state order
	if(msg->order.compare("force_state") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Forces a new transition to the specified status given as argument
		//int new_status = (int) msg->args[0];
		//hex->forceTransition(new_status);

	}

	//---pause order
	if(msg->order.compare("update_args") == 0){

		//Mode for checking functionality
		#ifdef DEBUG_MODE
		ROS_INFO("Received order: %s", msg->order.c_str());
		#endif

		//Copy the new args
		hex->args = parseargs2map(msg->args);

	}

	return;
}

//Friend function used to handle the reception of the pattern command as a separate
//thread
void clhero::handle_pattern_command_reception (Clhero_robot *clhr){

	//Ros required statements
	ros::NodeHandle nh;

	ros::Rate loop_rate (PATTERN_COMMAND_RECEPTION_RATE);

	//Subscription to the msg
	ros::Subscriber sub = nh.subscribe("pattern_command", 1000, clhero::handle_pattern_command_msg);

	//Main loop of the thread
	while(ros::ok()){

		//This thread shall only check for callbacks of the pattern command type
		//thus, checking for new msgs is the only task to fullfill on each loop
		//iteration

		//Spins for msgs
		ros::spinOnce();

		//Sleep until another iteration
		loop_rate.sleep();
	}
}

//Friend function that handles the reception of the leg status msg
void clhero::handle_leg_state_msg(const clhero_gait_controller::LegState::ConstPtr& msg){

	Clhero_robot* hex = instances_ptr[0];

	//Request the mutex for writing in the status
	leg_state_mutex.lock();

	//Writes
	hex->leg_state.pos = msg->pos;
	hex->leg_state.vel = msg->vel;
	hex->leg_state.torq = msg->torq;

	//Unlocks the mutex
	leg_state_mutex.unlock();

	return;
}

//Friend function used to handle the reception of the leg status as a separate
//thread
void clhero::handle_leg_state_reception (Clhero_robot* hex){
	//Ros required statements
	ros::NodeHandle nh;

	ros::Rate loop_rate (LEG_STATE_RECEPTION_RATE);

	//Subscription to the msg
	ros::Subscriber sub = nh.subscribe("legs_state_com", 1000, clhero::handle_leg_state_msg);

	//Main loop of the thread
	while(ros::ok()){

		//This thread shall only check for callbacks of the leg status type
		//thus, checking for new msgs is the only task to fullfill on each loop
		//iteration

		//Spins for msgs
		ros::spinOnce();

		//Sleep until another iteration
		loop_rate.sleep();
	}
}

//Friend function used to handle the active state threads
void clhero::handle_active_state(Clhero_robot* hex){

	//Launches the zero state
	hex->active_state_thread = new std::thread(hex->state_fcn_map[hex->active_state], hex);

	while(ros::ok()){

		#if DEBUG_MODE
		ROS_INFO("handle_active_state - about to join");
		#endif

		//Waits for the current state to find its end
		hex->active_state_thread->join();

		#if DEBUG_MODE
		ROS_INFO("handle_active_state - about to delete");
		#endif

		//set the new thread depending on the active state
		delete hex->active_state_thread;
		hex->active_state_thread = new std::thread(hex->state_fcn_map[hex->active_state], hex);
	}
}

//Function which is used as zero state for any gait pattern
void zero_state (Clhero_robot* hex){

	//The zero state shall do nothing and is used as a common starting point

	#ifdef DEBUG_MODE

	ROS_INFO("Entered state 0");

	#endif

	//Rate used to avoid an usage excess 
	ros::Rate loop_rate (50);

	//Stops all legs
	for(int i = 1; i<=LEG_NUMBER; i++){
		hex->setLegVelocity(i, 0);
	}

	//The zero state shall be active until 2 conditions are met:
	//	1. A starting state is set
	//	2. the status of the object is active
	while((hex->activeState() == 0)&&(ros::ok())){
		//Checks if the starting state is set
		if(hex->getStartingState() <= 0){

			#if DEBUG_MODE 
			ROS_INFO("no starting status is set");
			#endif

			loop_rate.sleep();

		}else{
			//If a starting state is set, shall request a transition
			if(hex->transition(hex->getStartingState())){

				#if DEBUG_MODE 
				ROS_INFO("Transition accepted, ending state zero");
				#endif

				return;
			}else{

				#if DEBUG_MODE 
				ROS_INFO("transition condition not met on state zero");
				#endif

				loop_rate.sleep();
			}
		} 
	}
}

//----------------------------------------------------
//    Class methods definition
//----------------------------------------------------

//Default constructor
Clhero_robot::Clhero_robot(std::string pattern_name){
	command_srv_client = nh.serviceClient<clhero_gait_controller::LegCommandRequest>("leg_command_request");
	gp_name = pattern_name;
	this->command_req.request.gait_pattern_name = gp_name;

	//Only one instance of the class shall be active at any time
	//as the thread that checks each receiving msg shall only be
	//launched once.
	if(instances_ptr.size() > 0){
		ROS_ERROR("Multiple declarations of Clhero_robot class.");
	}else{
		//adds the pointer to the vector
		instances_ptr.push_back(this);

		//Creates the thread for pattern command msg reception
		this->pattern_command_reception_thread = new std::thread(handle_pattern_command_reception, this);

		//Creates the thread for pattern command msg reception
		this->leg_state_reception_thread = new std::thread(handle_leg_state_reception, this);

		//Sets the initial status as stopped
		this->status = stop;

		//sets the id depending on the size of the instances vector
		this->id = instances_ptr.size();

		//Sets the active state as the zero state
		this->active_state = 0;
		this->attachState(0, zero_state);

		//Sets the default starting state as a negative valor
		this->starting_state = -1;

		//Launch the state handler thread
		this->state_handler_thread = new std::thread(handle_active_state, this);

		//Sets the commands as non buffered
		this->bufferCommands(false);
	}
}

//Set the position for a leg
void Clhero_robot::setLegPosition (int leg, 
					float position, 
					float velocity,
					bool new_acel_profile,
					float acel,
					float decel){

	//Checks if the pattern name is not the default.
	if(gp_name=="None"){
		ROS_INFO("No gait pattern name was set for node, command will not be sent.");
		return;
	}

	//Builds the msg
	for(int i=1; i<=LEG_NUMBER; i++){

		if(i==leg){

			this->is_new_command_req = true;

			this->command_req.request.position_command[i-1].data = true;

			this->command_req.request.new_command[i-1] = 1;
			this->command_req.request.pos[leg-1] = position;
			this->command_req.request.vel[leg-1] = velocity;

			if(new_acel_profile){
				this->command_req.request.new_acel_profile[leg-1].data = true;
				this->command_req.request.acel[leg-1] = acel;
				this->command_req.request.decel[leg-1] = decel;
			}else{
				this->command_req.request.new_acel_profile[leg-1].data = false;
				this->command_req.request.acel[leg-1] = 0;
				this->command_req.request.decel[leg-1] = 0;
			}
			
		}

	}

	//If the commands are not being buffered, sends the msg
	if(!are_commands_buffered){
		//Sends the msg
		this->sendCommands();
	}

	return;
}

//Set the position for multiple legs
void Clhero_robot::setLegPosition (std::vector<int> leg, 
					float position, 
					float velocity,
					bool new_acel_profile,
					float acel,
					float decel){

	//Checks if the pattern name is not the default.
	if(gp_name=="None"){
		ROS_INFO("No gait pattern name was set for node, command will not be sent.");
		return;
	}

	//Builds the msg
	for(int i=0; i<leg.size(); i++){

		this->is_new_command_req = true;

		this->command_req.request.new_command[leg[i]-1] = 1;

		this->command_req.request.position_command[leg[i]-1].data = true;
		this->command_req.request.pos[leg[i]-1] = position;
		this->command_req.request.vel[leg[i]-1] = velocity;

		if(new_acel_profile){
			this->command_req.request.new_acel_profile[leg[i]-1].data = true;
			this->command_req.request.acel[leg[i]-1] = acel;
			this->command_req.request.decel[leg[i]-1] = decel;
		}else{
			this->command_req.request.new_acel_profile[leg[i]-1].data = false;
			this->command_req.request.acel[leg[i]-1] = 0;
			this->command_req.request.decel[leg[i]-1] = 0;
		}

	}

	//If the commands are not being buffered, sends the msg
	if(!are_commands_buffered){
		//Sends the msg
		this->sendCommands();
	}

	return;
}

//Set the velocity for a leg
void Clhero_robot::setLegVelocity (int leg, 
					float velocity,
					bool new_acel_profile,
					float acel,
					float decel){

	//Checks if the pattern name is not the default.
	if(gp_name=="None"){
		ROS_INFO("No gait pattern name was set for node, command will not be sent.");
		return;
	}

	//Builds the msg
	for(int i=1; i<=LEG_NUMBER; i++){

		if(i==leg){

			this->is_new_command_req = true;

			this->command_req.request.position_command[i-1].data = false;

			this->command_req.request.new_command[i-1] = 1;
			this->command_req.request.pos[leg-1] = 0;
			this->command_req.request.vel[leg-1] = velocity;

			if(new_acel_profile){
				this->command_req.request.new_acel_profile[leg-1].data = true;
				this->command_req.request.acel[leg-1] = acel;
				this->command_req.request.decel[leg-1] = decel;
			}else{
				this->command_req.request.new_acel_profile[leg-1].data = false;
				this->command_req.request.acel[leg-1] = 0;
				this->command_req.request.decel[leg-1] = 0;
			}
			
		}

	}

	//If the commands are not being buffered, sends the msg
	if(!are_commands_buffered){
		//Sends the msg
		this->sendCommands();
	}

	return;
}

//Set the velocity for multiple legs
void Clhero_robot::setLegVelocity (std::vector<int> leg, 
					float velocity,
					bool new_acel_profile,
					float acel,
					float decel){

	//Checks if the pattern name is not the default.
	if(gp_name=="None"){
		ROS_INFO("No gait pattern name was set for node, command will not be sent.");
		return;
	}

	//Builds the msg
	for(int i=0; i<leg.size(); i++){

		this->is_new_command_req =  true;

		this->command_req.request.new_command[leg[i]-1] = 1;

		this->command_req.request.position_command[leg[i]-1].data = false;
		this->command_req.request.vel[leg[i]-1] = velocity;

		if(new_acel_profile){
			this->command_req.request.new_acel_profile[leg[i]-1].data = true;
			this->command_req.request.acel[leg[i]-1] = acel;
			this->command_req.request.decel[leg[i]-1] = decel;
		}else{
			this->command_req.request.new_acel_profile[leg[i]-1].data = false;
			this->command_req.request.acel[leg[i]-1] = 0;
			this->command_req.request.decel[leg[i]-1] = 0;
		}

	}

	//If the commands are not being buffered, sends the msg
	if(!are_commands_buffered){
		//Sends the msg
		this->sendCommands();
	}

	return;

}

//Controls if the command request shall be sent inmediately
//after the set function call or shall be buffered instead
//If true, once the set function is called, the msg is sent
void Clhero_robot::bufferCommands (bool c){

	this->are_commands_buffered = c;
	return;

}

//Clear the buffered command
//By default the initial command is a velocity command with 0
//as value for each leg.
void Clhero_robot::clearCommand(){

	for(int i=0; i<LEG_NUMBER; i++){
		this->command_req.request.pos[i] = 0;
		this->command_req.request.vel[i] = 0;
		this->command_req.request.acel[i] = 0;
		this->command_req.request.decel[i] = 0;
		this->command_req.request.new_acel_profile[i].data = false;
		this->command_req.request.position_command[i].data = false;
		this->command_req.request.new_command[i] = 0;
	}

	this->is_new_command_req = false;

	return;

}

//Sends the buffered command
bool Clhero_robot::sendCommands(){

	if(is_new_command_req){
		//Sends the msg
		if(command_srv_client.call(this->command_req)){
			//Clears the msg
			this->clearCommand();
			return true;
		}else{
			ROS_ERROR("Command service could not be called.");
			return false;
		}
	}
	
	return false;

}

//Function used to register new pattern name
void Clhero_robot::setGaitPatternName (std::string name){
	//Checks if the name is the default
	if(gp_name != DEFAULT_GP_NAME){
		//If different, gait pattern name is already set and shall avoid further changes
		//in order to avoid misuse of the library.
		ROS_ERROR("Gait Pattern name already set - name will not be changed");
	}else{
		gp_name = name;
		this->command_req.request.gait_pattern_name = gp_name;
	}
	return;
}

//Function to attach a new state
bool Clhero_robot::attachState (int state_id, void(*state_fcn)(Clhero_robot*), bool starting_state_flg){
	//Checks if the id is already in the map
	if(state_fcn_map.count(state_id)){
		//if is already at the map shall no attach the state
		ROS_INFO("Could not attach state");
		return false;
	}else{
		//If not, the state is attached
		this->state_fcn_map[state_id] = state_fcn;

		//If it is the starting state, the flag is set
		if(starting_state_flg){
			this->starting_state = state_id;
		}

		return true;
	}
}

//Function that returns the active state
int Clhero_robot::activeState (){
	return active_state;
}

//Function that returns the starting state
int Clhero_robot::getStartingState (){
	return starting_state;
}

//Function that forces a transition
bool Clhero_robot::forceTransition (int state_id){
	//checks that the id exists
	if(state_fcn_map.count(state_id) < 1){
		return false;
	}

	//Changes the active state
	this->active_state = state_id;
	return true;
}

//Function that performs the transition to a new state given as argument
bool Clhero_robot::transition (int state_id){
	//if the status is any different from active the transition shall fail
	if(status != active){

		#if DEBUG_MODE 
		ROS_INFO("[transition]: The status is not active, transition failed.");
		#endif

		return false;
	}

	//checks that the id exists
	if(state_fcn_map.count(state_id) < 1){

		#if DEBUG_MODE 
		ROS_INFO("[transition]: ID does not exist, transition failed.");
		#endif

		return false;
	}

	//if the transition could be done 
	#if DEBUG_MODE 
	ROS_INFO("[transition]: Transition success.");
	#endif
	this->active_state = state_id;

	return true;
}

//Function that lets the gait pattern run
void Clhero_robot::run (){
	this->state_handler_thread->join();
	return;
}

//Function that returns the state of the legs
LegState Clhero_robot::getLegState (){
	clhero::LegState ls;
	leg_state_mutex.lock();
	ls = this->leg_state;
	leg_state_mutex.unlock();
	return ls;
}

std::vector<float> Clhero_robot::getLegsPosition(){
	std::vector<float> ls;
	leg_state_mutex.lock();
	ls = this->leg_state.pos;
	leg_state_mutex.unlock();
	return ls;
}

std::vector<float> Clhero_robot::getLegsVelocity(){
	std::vector<float> ls;
	leg_state_mutex.lock();
	ls = this->leg_state.vel;
	leg_state_mutex.unlock();
	return ls;
}

std::vector<float> Clhero_robot::getLegsEffort(){
	std::vector<float> ls;
	leg_state_mutex.lock();
	ls = this->leg_state.torq;
	leg_state_mutex.unlock();
	return ls;
}

std::map<std::string, std::string> Clhero_robot::getArgs(){
	return this->args;
}

//Function usedo to register a new gait pattern on the parameter server
void clhero::registerGaitPattern(std::string name){

	//Node handle
	ros::NodeHandle nh;

	//Creates the client
	ros::ServiceClient client = nh.serviceClient<clhero_gait_controller::RegisterGaitPattern>("register_gait_pattern");

	//Creates the request
	clhero_gait_controller::RegisterGaitPattern msg;

	unsigned int count = 0;
	ros::Rate server_not_avaiable_loop_rate(REGISTRATION_SRV_NOT_AVAILABLE_LOOP_RATE);

	msg.request.pattern_name = name;

	//this register method shall call the registration service to avoid registration failure due
	//to gait manager not being started
	while((!client.call(msg)) && (count < REGISTRATION_SRV_NOT_AVAILABLE_TRY_LIMIT)){
		
		if (count == 0){
			std::string error_msg = "Could not call register_gait_pattern service for gait pattern " + name; 	
			ROS_ERROR(error_msg.c_str());
		}

		count++;

		server_not_avaiable_loop_rate.sleep();

		return;
	}

	if(!msg.response.ans){
		std::string emsg = "Pattern " + name + " could not be registered.";
		ROS_ERROR(emsg.c_str());
	}else{
		std::string succ_msg = "Registered gait pattern: " + name;
		ROS_INFO(succ_msg.c_str());
	}

  	return;
	
}