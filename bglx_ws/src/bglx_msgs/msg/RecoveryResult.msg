# Sent back when a recovery attempt finishes, successfully or not.

std_msgs/Header header

# Did the vehicle end up somewhere it can drive from?
bool freed

# How far it actually moved during recovery
float32 moved_distance

# Tool calls consumed, against the budget
uint8 tool_calls_used

# The recovering agent's judgement on the original goal. False means do not
# retry it - blacklist and move on. This is deliberately the agent's call and
# not the explorer's, because the agent has just been there and looked.
bool goal_worth_retrying

# Plain-language account of what was done and why, for the log
string account
