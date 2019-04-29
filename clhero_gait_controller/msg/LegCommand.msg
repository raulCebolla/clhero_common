#================================================================
# LegCommand.msg
#
# Description: this message contains the command for every leg
#================================================================

#Position
float32[6] pos

#Velocity
float32[6] vel

#Acceleration
float32[6] acel

#Decceleration
float32[6] decel

#Flag indicating if a new acceleration profile is set
std_msgs/Bool[6] new_acel_profile

#Flag indicating if the command is position-type
std_msgs/Bool[6] position_command