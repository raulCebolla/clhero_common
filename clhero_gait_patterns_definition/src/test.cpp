
#include <ros/ros.h>
#include <clhero_gait_controller/clhero.h>
#include <string>

//Estado 1
void state_1 (clhero::Clhero_robot* clhr){

  ROS_INFO("Entered state 1");

  ros::Rate loop_rate(50);

  for(int i=0; i<6; i++){
    clhr->setLegPosition(i+1, 90);
  }

  while(clhr->activeState() == 1){
    loop_rate.sleep();
  }

  return;
}

//Estado 2
void state_2 (clhero::Clhero_robot* clhr){
  
  ROS_INFO("Entered state 2");

  ros::Rate loop_rate(50);

  for(int i=0; i<6; i++){
    clhr->setLegPosition(i+1, -90);
  }
  
  while(clhr->activeState() == 2){
    loop_rate.sleep();
  }
  return;
}

int main (int argc, char** argv){

	ros::init(argc, argv, "test_sub");
	ros::NodeHandle nh;

	std::string pattern_name = "test_gait_pattern";

	clhero::Clhero_robot clhr (pattern_name);

	//Register the new gait pattern
	clhero::registerGaitPattern(pattern_name);

	//try to change the name
	//ROS shall give an error msg due to this sentence
	clhr.setGaitPatternName("another");

  //attach the states set
  clhr.attachState(1, state_1, STARTING_STATE);
  clhr.attachState(2, state_2);

  //Lets the gait pattern run
  clhr.run();

	
	return 0;

}