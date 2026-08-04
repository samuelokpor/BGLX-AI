# Sent by the frontier explorer when it detects that the vehicle has stopped
# making progress and the fault is positional rather than a bad goal.
#
# The explorer can only speak in goals: when a plan fails it picks a different
# frontier, which fails identically because the ROBOT is the problem, not the
# destination. This message hands that situation to something that can
# reposition the vehicle.

std_msgs/Header header

# Why recovery was asked for: "stalled", "aborted", "no_path"
string reason

# Where the vehicle is now, in the global frame
float64 x
float64 y
float64 yaw

# What it was trying to reach when it stopped making progress
float64 goal_x
float64 goal_y

# Nav2's account of the failure, already translated into diagnosis text
string diagnosis

# Sector distances at the moment of the stall
string scan_summary

# How badly it stalled
float32 moved_distance
float32 stalled_seconds
