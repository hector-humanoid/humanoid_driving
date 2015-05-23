# Cease all operation, go to safe pose
std_msgs/Bool all_stop

# Press throttle pedal?
std_msgs/Bool drive_forward

# Target steering angle ([0,...,360])
std_msgs/Float64 absolute_steering_angle

# time_from_start for trajectory msgs - DEBUG ONLY, will be removed
std_msgs/Float64 time_from_start
