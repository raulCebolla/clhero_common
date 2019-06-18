#================================================================
# pattern_command.msg
#
# Description: this message is sent in order to command and set
# different arguments and orders to each of the declared gait
# patterns
#================================================================

#Name of the gait pattern

string pattern_name

#Arguments
#In this variable shall be set arguments such as the velocity or
#the legs contact angle.

string args

#Order
#	- stop: stop the gait pattern at an initial or emergency state
#	- pause: stop the gait pattern at the current state
#	- continue:  resumes the course of action
#	- force_state: transitions to a defined state given in args
#	- start: starts the gait pattern
#	- update_args: update the gait pattern's arguments

string order