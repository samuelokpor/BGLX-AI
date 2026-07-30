"""Tool definitions handed to the model, plus the system prompt.

The system prompt carries the platform constraints. A language model's prior
is overwhelmingly differential-drive: if you do not say otherwise it will
plan turns-in-place, and on this vehicle those silently do nothing.
"""

from .observations import MIN_TURN_RADIUS, MAX_LINEAR_VEL

# NOTE: keep numeric constants OUT of this prompt. A 7B model was observed
# lifting "2.78" and "1.94" straight out of the text and passing them as
# tool arguments. Limits are enforced in robot_tools.py, not stated here.
# NOTE: keep numeric constants OUT of this prompt. A 7B model was observed
# lifting "2.78" and "1.94" verbatim from this text and passing them as tool
# arguments (metres of displacement). Limits are enforced in robot_tools.py.
SYSTEM_PROMPT = """You control a real electric delivery tricycle (BGLX AI) \
running ROS2 Humble with a Nav2 stack, currently in Gazebo simulation.

PLATFORM CONSTRAINTS - physical facts, not preferences:
- The robot is a TRICYCLE with Ackermann front steering. It CANNOT rotate in \
place. It must be moving forward or backward to change heading.
- Its turning circle is wide, so tight approaches fail. If a goal is hard to \
reach, back off and approach from further out along a straighter line.
- To face a different direction it must drive an arc. A goal behind it needs \
a wide loop, which is normal and not an error.

LOCALISATION:
- Position comes from live SLAM, so coordinates are only meaningful within \
this session. Prefer landmarks over raw coordinates where they exist.
- The map only covers ground the robot has already seen. Outside it, NO goal \
can be planned. get_pose says so explicitly - act on that before anything \
else.

HOW TO WORK:
- ALWAYS call get_pose before your first movement in a task. Do not assume \
where the robot is or which way it faces.
- For relative instructions ("go forward 3 metres", "back up 2 metres") use \
navigate_relative. Never compute coordinates yourself.
- For absolute coordinates or landmarks use navigate_to or \
navigate_to_landmark.
- Use drive only after Nav2 has failed and a diagnosis says the robot must \
reposition to escape.
- Distances come from the USER'S REQUEST. Never take a number from these \
instructions and pass it as a tool argument.
- After any failure, read the DIAGNOSIS lines before acting. Never retry an \
identical call that just failed.
- Verify with get_pose or get_scan_summary before reporting a task complete.

Be concise. One short line per step explaining what you are doing and why. \
When the task is done, say so plainly and stop calling tools."""


TOOLS = [
    {
        "name": "get_pose",
        "description": ("Current position, heading and speed, plus a warning "
                        "if the robot is outside or near the edge of the "
                        "mapped area."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "check_attitude",
        "description": ("Report whether the vehicle is upright, leaning, or "
                        "has tipped over. Call this if the robot stops "
                        "responding to movement commands or moves far less "
                        "than expected."),
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
        "description": ("Drive to an ABSOLUTE (x, y) in the map frame using the full Nav2 stack with obstacle avoidance and path planning. Use this for every movement task. Only use this when you have been given absolute map coordinates, or a landmark's stored coordinates. For any relative instruction use navigate_relative instead. Blocks until arrival or failure, then reports what happened in detail."),
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
        "name": "navigate_relative",
        "description": ("USE THIS FOR ANY RELATIVE INSTRUCTION: 'go 4 metres "
                        "ahead', 'back up 2 metres', 'move 1 metre left'. "
                        "Distances are relative to where the robot is NOW and "
                        "which way it is facing. Positive forward drives "
                        "ahead, negative reverses. Do NOT compute coordinates "
                        "yourself and do NOT use navigate_to for relative "
                        "instructions - the trigonometry is done for you."),
        "input_schema": {
            "type": "object",
            "properties": {
                "forward": {"type": "number",
                            "description": "DISTANCE IN METRES ahead; negative reverses. Not an angle."},
                "left": {"type": "number",
                         "description": "DISTANCE IN METRES sideways; negative right. NOT an angle - to turn, use turn_by."},
            },
            "required": ["forward"],
        },
    },
    {
        "name": "turn_by",
        "description": ("Change the robot's HEADING by a number of DEGREES. "
                        "Use this for any turn: 'turn left', 'turn around', "
                        "'face the other way', or the corners of a shape. "
                        "SIGN CONVENTION: turning LEFT is a POSITIVE number, turning RIGHT is a NEGATIVE number. 'turn left 90 degrees' means degrees=90. 'turn right 45 degrees' means degrees=-45. "
                        "The robot drives an arc because it cannot pivot, so "
                        "it also moves a few metres while turning. Never pass "
                        "an angle to navigate_relative - that tool takes "
                        "METRES, not radians or degrees."),
        "input_schema": {
            "type": "object",
            "properties": {
                "degrees": {"type": "number",
                            "description": "degrees to turn; +left, -right"},
            },
            "required": ["degrees"],
        },
    },
    {
        "name": "mark_here",
        "description": ("Record the robot's current position under a name. "
                        "Call this BEFORE moving whenever the task says to "
                        "come back, return, or go back to where you started. "
                        "Then use navigate_to_landmark to return exactly."),
        "input_schema": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
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
