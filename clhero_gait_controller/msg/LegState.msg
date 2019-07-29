#================================================================
# LegState.msg
#
# Description: this message contains the state of each one of 
# the legs.
#================================================================

#Time stamp in which the state was taken
time stamp

#Legs'position [rad]
float32[] pos

#Leg's velocity [rad/s]
float32[] vel

#Leg's torque [Nm]
float32[] torq