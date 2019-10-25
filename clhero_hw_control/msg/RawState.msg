#=====================================================
#
#	RawState.msg
#
#	Description: Msg that publish raw information 
#	about the state of the legs directly read from 
#	the maxon motors
#=====================================================

time stamp

#Position [encoder pulses]
float64[] pos

#Velocity [rpm]
float64[] vel

#Current []
float64[] eff