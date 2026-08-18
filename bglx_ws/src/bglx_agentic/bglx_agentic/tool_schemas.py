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
- For a complete parcel delivery between named mission locations, use run_delivery_mission. Do NOT manually sequence navigate_to calls for pickup, delivery and return-home legs. Call list_delivery_locations first if the location names are uncertain. \
- When the user asks to remember, save, teach, or record the CURRENT place specifically as a delivery pickup/drop-off/depot, use record_delivery_location. Do not use the generic mark_here tool for delivery locations. \
- When the user explicitly asks to move/update/correct an EXISTING saved delivery location to the robot's CURRENT position, use update_delivery_location. Do not use record_delivery_location for an existing location. \
- When the user explicitly asks to forget/delete/remove a saved CUSTOM delivery location, use delete_delivery_location. Never delete a location merely because it is unused or inconvenient. \
- run_delivery_mission owns the complete delivery state machine and its controlled retries. Once it is running, do not duplicate its navigation legs with other movement tools. \
- Do NOT manually navigate to HOME before run_delivery_mission. The mission tool performs any required return-to-HOME preparation itself. \
- Use drive only after Nav2 has failed and a diagnosis says the robot must \
reposition to escape.
- Distances come from the USER'S REQUEST. Never take a number from these \
instructions and pass it as a tool argument.
- After any failure, read the DIAGNOSIS lines before acting. Never retry an \
identical call that just failed.
- Verify with get_pose or get_scan_summary before reporting a task complete.


MISSION CONTROL:
- run_delivery_mission STARTS the deterministic delivery asynchronously. A \
successful tool call means the mission started, NOT that the parcel has been \
delivered.
- While a delivery mission is active, NEVER issue navigate_to, \
navigate_relative, navigate_to_landmark, turn_by, or drive. The mission \
controller owns vehicle movement.
- Use get_mission_status when asked for progress. Only say the delivery is \
complete when get_mission_status reports COMPLETE or the mission controller \
reports MISSION_COMPLETE.
- Use cancel_delivery_mission if the user asks to cancel an active delivery. \

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
        "name": "check_low_obstacles",
        "description": ("Check for LOW obstacles ahead using the front LiDAR "
                        "mounted at 0.35m. The main LiDAR sits at 0.98m and "
                        "sees straight over anything shorter than that - "
                        "kerbs, boxes, planters, luggage, a crouching child. "
                        "This is the ONLY sensor that detects them. Call this "
                        "before moving forward in any unfamiliar or cluttered "
                        "place, and whenever the path looks clear but you are "
                        "not certain the ground is. A BLOCKED result here "
                        "outranks a clear costmap or a clear scan summary."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "check_map_against_sensors",
        "description": ("Compare the saved map with what the LiDAR sees right "
                        "now. Reports things PRESENT that the map does not "
                        "know about (a parked vehicle, a delivery, a barrier) "
                        "and things GONE that the map still records. Call this "
                        "before committing to a narrow route, and whenever "
                        "navigation fails in a place the map says is clear. "
                        "Also reports how confident the localisation is - a "
                        "mismatch while poorly localised usually means the "
                        "robot is lost, not that the world has changed."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "check_width",
        "description": ("Measure the gap in a given direction and say whether "
                        "the vehicle fits. Call this BEFORE entering any "
                        "archway, doorway, alley or narrow passage. This "
                        "platform cannot rotate in place, so if it enters "
                        "somewhere too narrow it cannot turn around and gets "
                        "stranded - the check is worthless once you are "
                        "already inside."),
        "input_schema": {
            "type": "object",
            "properties": {
                "direction_deg": {"type": "number",
                                  "description": "relative to current heading: 0 ahead, 90 left, -90 right, 180 behind"},
            },
        },
    },
    {
        "name": "check_turn_around",
        "description": ("Is there room to reverse direction here? The vehicle "
                        "needs about 1.6m of clear space in every direction "
                        "because it cannot pivot. Ask before committing to "
                        "anywhere constrained."),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "look",
        "description": ("Look through the front camera and get a description "
                        "of WHAT is ahead, not just how far. Use this when "
                        "the LiDAR reports an obstacle and the right response "
                        "depends on what it is: a person who will move, a van "
                        "unloading, cones marking a closure, or a permanent "
                        "fixture. Also use it before reporting that a route "
                        "is blocked. Slower than get_scan_summary and gives "
                        "no distances - pair the two."),
        "input_schema": {
            "type": "object",
            "properties": {
                "question": {"type": "string",
                             "description": "optional specific question about the scene"},
            },
        },
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
        "description": ("Drive to a previously recorded named location in ONE "
                        "call. Use this whenever the task mentions going back, "
                        "returning, or a place name - 'go home', 'back to the "
                        "dock', 'return to the loading bay'. Do NOT step "
                        "backwards with navigate_relative to reach a landmark; "
                        "Nav2 plans the whole route for you."),
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
        "name": "record_delivery_location",
        "description": (
            "Save the robot's CURRENT map position as a persistent BGLX "
            "delivery location. Use this when the user is physically at a "
            "place and asks to remember it as a pickup, delivery/drop-off, "
            "or depot. The location survives agent restarts."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": (
                        "Human-readable location name, for example "
                        "Building B or Warehouse A."
                    ),
                },
                "location_type": {
                    "type": "string",
                    "enum": [
                        "pickup",
                        "delivery",
                        "depot",
                    ],
                    "description": (
                        "Role of this location in delivery missions."
                    ),
                },
                "aliases": {
                    "type": "array",
                    "items": {
                        "type": "string",
                    },
                    "description": (
                        "Optional alternative natural-language names."
                    ),
                },
            },
            "required": [
                "name",
                "location_type",
            ],
        },
    },

    {
        "name": "update_delivery_location",
        "description": (
            "Explicitly update an EXISTING user-defined BGLX delivery "
            "location so its coordinates become the robot's CURRENT map "
            "position. Use only when the user asks to update, move, or "
            "correct an already saved delivery location. Built-in locations "
            "cannot be updated. Existing type and aliases are preserved."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": (
                        "Existing saved custom delivery location name "
                        "or alias, for example Building B."
                    ),
                },
            },
            "required": [
                "name",
            ],
        },
    },
    {
        "name": "delete_delivery_location",
        "description": (
            "Delete/forget an EXISTING user-defined BGLX delivery "
            "location. Use ONLY when the user explicitly asks to forget, "
            "delete, or remove that saved location. Built-in locations "
            "such as HOME cannot be deleted."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": (
                        "Existing saved custom delivery location name "
                        "or alias to forget."
                    ),
                },
            },
            "required": [
                "name",
            ],
        },
    },

    {
        "name": "list_delivery_locations",
        "description": (
            "List BGLX delivery locations including their canonical names, "
            "location types and friendly aliases. Use this when the user "
            "refers to a pickup or delivery place by a natural name."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "run_delivery_mission",
        "description": (
            "START a COMPLETE autonomous parcel delivery using named mission "
            "locations. The deterministic mission controller drives to the "
            "pickup, performs the loading state, drives to the delivery "
            "location, unloads, and returns HOME. It also owns controlled "
            "navigation retries and abort handling. This tool returns after ""the mission has STARTED; it does not mean the delivery is complete. ""Use get_mission_status for progress. Use this instead of "
            "manually issuing navigate_to calls for each delivery leg. "
            "If the vehicle is away from HOME, the mission tool automatically "
            "returns it to HOME before starting the pickup leg."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "pickup": {
                    "type": "string",
                    "description": (
                        "Pickup location name or alias, for example PICKUP_A, "
                        "pickup a, or loading point a."
                    ),
                },
                "delivery": {
                    "type": "string",
                    "description": (
                        "Delivery location name or alias, for example DELIVERY_A, "
                        "delivery a, or drop-off a."
                    ),
                },
            },
            "required": [
                "pickup",
                "delivery",
            ],
        },
    },
    {
        "name": "get_mission_status",
        "description": (
            "Report the current deterministic delivery mission state. "
            "Use this after run_delivery_mission or whenever the user asks "
            "whether a delivery is still running, complete, stopped, "
            "loading, unloading, or returning home."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "cancel_delivery_mission",
        "description": (
            "Cancel the currently active deterministic delivery mission. "
            "This interrupts the mission controller, cancels its active "
            "Nav2 goal, and stops mission execution. Use only when the user "
            "requests cancellation or when continuing the mission is unsafe."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
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
