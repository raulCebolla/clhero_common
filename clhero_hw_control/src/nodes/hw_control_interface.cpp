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
#define CONTROL_RATE 100 //Rate at which the node sends the control of each leg
#define PI 3.14159265359
#define ANG_THR 0.03490658503988659 //2*(2*PI)/360
#define ANG_V 0.17453292519943295 //10*(2*PI)/360
#define STATE_UPDATE_RATE 200 //Rate at which the state of the legs is update [Hz]

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

//Class that keeps the info of the last msg
class Command {

  public:

    float pos[LEG_NUMBER];
    float vel[LEG_NUMBER];
    float acel[LEG_NUMBER];
    float decel[LEG_NUMBER];
    bool new_acel_profile[LEG_NUMBER];
    bool position_command[LEG_NUMBER];

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
    void updateCommand (const clhero_gait_controller::LegCommand::ConstPtr& msg){
      for(int i=0; i<LEG_NUMBER; i++){
        if( (pos[i] != msg->pos[i]) || (vel[i] != msg->vel[i]) ){
          pos[i] = msg->pos[i];
          vel[i] = msg->vel[i];
          acel[i] = msg->acel[i];
          decel[i] = msg->decel[i];
          new_acel_profile[i] = msg->new_acel_profile[i].data;
          position_command[i] = msg->position_command[i].data;
        }
      }
      return;
    }

};

struct Leg_state {
  float position [LEG_NUMBER];
  float velocity [LEG_NUMBER];
  float effort [LEG_NUMBER];
};

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Publisher of the legs_state_msg
ros::Publisher legs_state_pub;

//Publisher of each of the command msgs
std::vector<ros::Publisher> controller_command_pub;

//Legs'command
Command command;

//Legs'command mutex
std::mutex command_mtx;

//State of the legs
Leg_state leg_state;

//Legs state mutex
std::mutex leg_state_mtx;

epos_functions* epos;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  return (angle - 2*PI*trunc(angle/(2*PI)));
}

//Thread for each leg command control
/*void control_leg (int leg){

  //Rate of the control
  ros::Rate loop_rate (CONTROL_RATE);

  //Message to be sent
  std_msgs::Float64 msg;

  //Movement control variables
  double ang, ang_ref, vel_ref;

  while(ros::ok()){
    //The behaviour of the control shall change whether it is a position command or not
    if(command.position_command[leg-1]){
      //Position command
      
      leg_state_mtx.lock();
      ang = leg_state.position[leg-1];
      leg_state_mtx.unlock();

      command_mtx.lock();
      ang_ref = command.pos[leg-1];
      vel_ref = command.vel[leg-1];
      command_mtx.unlock();

      msg.data = posControlVel(ang, ang_ref, vel_ref);

    }else{
      //Velocity command
      //In velocity command, just the velocity shall be sent
      command_mtx.lock();
      msg.data = command.vel[leg-1];
      command_mtx.unlock();
    }

    controller_command_pub[leg-1].publish(msg);

    loop_rate.sleep();
  }

  return;
  
}
*/

//Callback for joint states msgs
/*void jointStatesCallback (const sensor_msgs::JointState::ConstPtr& msg){

  clhero_gait_controller::LegState leg_state_msg;

  leg_state_mtx.lock();
  for(int i=0; i < LEG_NUMBER; i++){
    leg_state_msg.pos.push_back(fixAngle(msg->position[i]));
    leg_state_msg.vel.push_back(msg->velocity[i]);
    leg_state_msg.torq.push_back(msg->effort[i]);
    leg_state.position[i] = leg_state_msg.pos[i];
    leg_state.velocity[i] = leg_state_msg.vel[i];
    leg_state.effort[i] = leg_state_msg.torq[i];
  }
  leg_state_mtx.unlock();

  legs_state_pub.publish(leg_state_msg);

  return;
}
*/

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

  //Repeats for each of the legs
  for(int i=0; i<LEG_NUMBER; i++){
  	
  	//First checks the mode: position or velocity
		if(msg->position_command[i].data){
			//Position command

			//Checks if a new profile shall be set
			if(msg->new_acel_profile[i].data){
				//If so, uploads the acceleration and decceleration
				epos->SetPositionProfile(i+1, (int)rint(msg->vel[i]), (int)rint(msg->acel[i]), (int)rint(msg->decel[i]));
			}

			//Once the position has been set sends the order
			epos->MoveToPosition(i+1, (int)rint(msg->pos[i]), TRUE, TRUE);
		}else{
			//Velocity command

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

	//Creates a node handle to publish the msg
	ros::NodeHandle nh;

	//Rate in which the state is updated
	ros::Rate state_update_rate (STATE_UPDATE_RATE);

	//Core loop of the thread
	while(ros::ok()){
		
	}
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
  /*
  for(int i=0; i < LEG_NUMBER; i++){
    controller_command_pub.push_back(nh.advertise<std_msgs::Float64>(controller_namespace + "/motor" + std::to_string(i+1) + "_velocity_controller/command", 1000));
  }
  */
  
  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 1000, legCommandCallback);
  //ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1000, jointStatesCallback);
  

  //threads with the control of each leg
  /*
  std::thread control_leg_1_thr (control_leg, 1);
  std::thread control_leg_2_thr (control_leg, 2);
  std::thread control_leg_3_thr (control_leg, 3);
  std::thread control_leg_4_thr (control_leg, 4);
  std::thread control_leg_5_thr (control_leg, 5);
  std::thread control_leg_6_thr (control_leg, 6);
  */

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  /*
  control_leg_1_thr.join();
  control_leg_2_thr.join();
  control_leg_3_thr.join();
  control_leg_4_thr.join();
  control_leg_5_thr.join();
  control_leg_6_thr.join();
  */

  delete epos;

  return 0;

}
