# Represent Actuator Outputs.
#
# ACTUATOR_OUTPUTS message

std_msgs/Header header

uint32 noutputs				# valid outputs
float32[16] output				# output data, in natural output units
