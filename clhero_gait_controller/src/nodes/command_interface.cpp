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
#include <std_msgs/String.h>
#include <clhero_gait_controller/LegCommand.h>
#include <clhero_gait_controller/LegCommandRequest.h>
#include <clhero_gait_controller/ChangeGaitPattern.h>
#include <clhero_gait_controller/LegState.h>
#include <cstring>
#include <map>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <clhero_gait_controller/clhero.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define COMMAND_MSG_RATE 50
#define LEG_STATE_MSG_RATE 100
#define CURRENT_GP_PARAM_NAMESPACE "/clhero/gait_patterns/current"
#define REGISTERED_GP_PARAM_NAMESPACE "/clhero/gait_patterns/registered"
#define LEG_NUMBER 6

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

//Class that keeps the info of the last msg
class Command {

    float pos[LEG_NUMBER];
    float vel[LEG_NUMBER];
    float acel[LEG_NUMBER];
    float decel[LEG_NUMBER];
    bool new_acel_profile[LEG_NUMBER];
    bool position_command[LEG_NUMBER];

  public:

    //Default constructor
    Command (){
      for (int i = 0; i<LEG_NUMBER; i++){
        pos[i] = 0;
        vel[i] = 0;
        acel[i] = 0;
        decel[i] = 0;
        new_acel_profile[i] = false;
        return;
      }
    }

    //Method that updates the com
    void updateCommand (clhero_gait_controller::LegCommandRequest::Request msg){
      for(int i=0; i<LEG_NUMBER; i++){
        if(msg.new_command[i]){
          pos[i] = msg.pos[i];
          vel[i] = msg.vel[i];
          acel[i] = msg.acel[i];
          decel[i] = msg.decel[i];
          new_acel_profile[i] = msg.new_acel_profile[i].data;
          position_command[i] = msg.position_command[i].data;
        }
      }
      return;
    }

    //Method that builds the msg to be sent
    clhero_gait_controller::LegCommand writeMsg (){
      clhero_gait_controller::LegCommand msg;

      for(int i = 0; i<6; i++){
        msg.pos[i] = pos[i];
        msg.vel[i] = vel[i];
        msg.acel[i] = acel[i];
        msg.decel[i] = decel[i];
        msg.new_acel_profile[i].data = new_acel_profile[i];
        new_acel_profile[i] = false;
        msg.position_command[i].data = position_command[i]; 
      }
      return msg;
    }

};

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Global variable that sets the node to debug mode
int debug_mode;

//Current gait pattern 
std::string current_gait_pattern;

//Last command to be sent
Command com;

//Mutex 
std::mutex mtx;

//Leg's state
clhero::LegState legs_state;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function which checks and sets the debug mode
bool checkDebugMode(int argc, char** argv){

  debug_mode = 0;

  for(int i = 0; i < argc; i++){
    if(!strcmp(argv[i], "debug")){
      ROS_INFO("Entering debug mode.");
      debug_mode = 1;
      return true;
    }
  }

  return false;
}

//Service callback for changing current gait pattern
bool changeGP (clhero_gait_controller::ChangeGaitPattern::Request &req,
               clhero_gait_controller::ChangeGaitPattern::Response &res){

  current_gait_pattern = req.name;
  res.ans = 0;
  return true;

}


//Service callback for sending new command
bool newCommandCallback (clhero_gait_controller::LegCommandRequest::Request &req,
                         clhero_gait_controller::LegCommandRequest::Response &res){
  if(req.gait_pattern_name!=current_gait_pattern){
    res.ans = 1;
    return true;
  }

  com.updateCommand(req);
  res.ans = 0;
  return true;
}


//Routine for periodical msg delivery
void commandThread (ros::Publisher* pub_leg_command){

  //Creation of a rate for scheduling the msg generation
  ros::Rate loop_rate(COMMAND_MSG_RATE); //Arg is frequency in Hz

  //msg command
  clhero_gait_controller::LegCommand msg;

  while(ros::ok()){

    //Writes the msg
    msg = com.writeMsg();

    pub_leg_command->publish(msg);
    
    //Sleeps for the desired rate
    loop_rate.sleep();
  }

  return;
}

//Routine for periodical leg status msg delivery
void legStateThread (ros::Publisher* pub_leg_state){

  //Creation of a rate for scheduling the msg generation
  ros::Rate loop_rate(LEG_STATE_MSG_RATE); //Arg is frequency in Hz

  //msg command
  clhero_gait_controller::LegState msg;

  while(ros::ok()){

    mtx.lock();

    //Writes the msg
    msg.pos = legs_state.pos;
    msg.vel = legs_state.vel;
    msg.torq = legs_state.torq;

    mtx.unlock();

    pub_leg_state->publish(msg);
    
    //Sleeps for the desired rate
    loop_rate.sleep();
  }

  return;
}

//Function that handles the reception of a leg state msg
void leg_state_msg_callback(const clhero_gait_controller::LegState::ConstPtr& msg){

  //Reads the msg and updates the legs'state
  mtx.lock();
  legs_state.pos = msg->pos;
  legs_state.vel = msg->vel;
  legs_state.torq = msg->torq;
  mtx.unlock();

  return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  checkDebugMode(argc, argv);

  //Legs'state initialization
  for(int i=0; i<6; i++){
    legs_state.pos.push_back(0);
    legs_state.vel.push_back(0);
    legs_state.torq.push_back(0);
  }

  //Creates and sets the gait_pattern/current parameter to none until one is loaded
  nh.setParam(CURRENT_GP_PARAM_NAMESPACE, "None");

  //Creation of the publisher for the msg describing the command
  ros::Publisher pub_leg_command = nh.advertise<clhero_gait_controller::LegCommand>("legs_command", 1000);

  //Creation of the publisher for the msg describing the command
  ros::Publisher pub_leg_state = nh.advertise<clhero_gait_controller::LegState>("legs_state_com", 1000);

  //Creation of the subscriber for the msg describing the legs'status
  ros::Subscriber state_sub = nh.subscribe("legs_state", 1000, leg_state_msg_callback);

  //Creation of the server for the change gait pattern service
  ros::ServiceServer change_gp_service = nh.advertiseService("change_gait_pattern", changeGP);

  //Creation of the server for the command service
  ros::ServiceServer com_req_service = nh.advertiseService("leg_command_request", newCommandCallback);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------
  std::thread command_thread (commandThread, &pub_leg_command);
  std::thread state_thread (legStateThread, &pub_leg_state);

  ros::spin();
  command_thread.join();
  state_thread.join();

  return 0;

}
