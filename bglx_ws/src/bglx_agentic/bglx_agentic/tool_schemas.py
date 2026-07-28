"""Tool definitions handed to the model, plus the system prompt.

The system prompt carries the platform constraints. A language model's prior
is overwhelmingly differential-drive: if you do not say otherwise it will
plan turns-in-place, and on this vehicle those silently do nothing.
"""

from .observations import MIN_TURN_RADIUS, MAX_LINEAR_VEL

SYSTEM_PROMPT = """You control a real electric delivery tricycle (BGLX AI) \
running ROS2 Humble with a Nav2 stack, currently in Gazebo simulation.

PLATFORM CONSTRAINTS - these are physical facts, not preferences:
- The robot is a TRICYCLE with Ackermann front steering. It CANNOT rotate in \
place. Minimum turning radius is %.2f metres.
- Yaw rate is proportional to forward speed. At zero speed, no amount of \
commanded rotation changes the heading.
- Maximum speed is %.2f m/s.
- Because of the turning circle, tight approaches fail. If a goal is hard to \
reach, back off and approach from further out along a straighter line.
- To face a different direction the robot must drive an arc. A goal behind it \
requires a wide loop, which is normal and not an error.

LOCALISATION:
- Position comes from live SLAM, so coordinates are only meaningful within \
this session. Prefer landmarks over raw coordinates where they exist.
- The map only covers ground the robot has already seen. If the robot leaves \
the mapped area, NO goal can be planned until it returns. get_pose will say \
so explicitly - believe it and act on it before trying anything else.

HOW TO WORK:
- Prefer navigate_to or navigate_to_landmark. They use the full Nav2 stack \
with obstacle avoidance. Use drive only as an escape hatch after Nav2 has \
failed, or for small repositioning.
- After any failure, read the DIAGNOSIS lines in the result before acting. \
Never retry an identical command that has just failed.
- Call get_pose and get_scan_summary when you need world state. Do not assume \
it or carry it over from earlier in the task.
- Sector distances come from a LIDAR that is blind within 0.45m. Something \
very close may not appear at all.

Be concise. One short line per step explaining what you are doing and why. \
When the task is complete, say so plainly and stop calling tools.""" % (
    MIN_TURN_RADIUS, MAX_LINEAR_VEL)


TOOLS = [
    {
        "name": "get_pose",
        "description": ("Current position, heading and speed, plus a warning "
                        "if the robot is outside or near the edge of the "
                        "mapped area."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_scan_summary",
        "description": ("Nearest obstacle distance in each of eight sectors "
                        "around the robot, from the LIDAR."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_landmarks",
        "description": "List named locations recorded in this map.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "navigate_to",
        "description": ("THE PRIMARY WAY TO MOVE. Drive to an absolute (x, y) in the map frame using the full Nav2 stack with obstacle avoidance and path planning. Use this for every movement task. For a RELATIVE move such as \"go 4 metres ahead\", first call get_pose, then compute the absolute target from the current position and heading. Blocks until arrival or failure, then reports what happened in detail."),
        "input_schema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "metres, map frame"},
                "y": {"type": "number", "description": "metres, map frame"},
                "yaw": {"type": "number",
                        "description": "desired final heading in radians"},
            },
            "required": ["x", "y"],
        },
    },
    {
        "name": "navigate_to_landmark",
        "description": "Drive to a previously recorded named location.",
        "input_schema": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
        },
    },
    {
        "name": "drive",
        "description": ("DANGEROUS LAST RESORT. Blind open-loop motion with NO obstacle avoidance and NO collision checking - the robot will drive into walls and people. Do NOT use this to travel to a destination; use navigate_to for that, always. Only valid after navigate_to has already failed and the diagnosis says the robot must reposition to escape. Keep speed at or below 0.5 m/s. Negative linear_x reverses."),
        "input_schema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "m/s"},
                "angular_z": {"type": "number", "description": "rad/s"},
                "duration": {"type": "number",
                             "description": "seconds, max 10"},
            },
            "required": ["linear_x", "angular_z", "duration"],
        },
    },
    {
        "name": "wait",
        "description": "Hold position for a number of seconds.",
        "input_schema": {
            "type": "object",
            "properties": {"seconds": {"type": "number"}},
            "required": ["seconds"],
        },
    },
    {
        "name": "get_last_failure",
        "description": ("Full diagnosis of the most recent navigation "
                        "failure. Read before retrying anything."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "stop",
        "description": "Immediately halt the robot.",
        "input_schema": {"type": "object", "properties": {}},
    },
]
