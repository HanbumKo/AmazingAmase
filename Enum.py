# drone state
STATE_ALIVE = 0
STATE_DEAD = 1

# what it is doing now
INITIAL_STATE = -1
SEARCHING = 0x01
TRACKING = 0x10
UNDERTAKED = 0x100
CHARGING = 0x1000
DEAD = 0x10000
TURNING = 0x100000

# program phase
PHASE_SETTING = 0
PHASE_UPDATE = 1

# initialsearch way
INIT_START_NEAREST = 0
INIT_START_FARTHEST = 1
INIT_START_SMALLEST = 2
INIT_START_LARGEST = 3

# Command_ID
CMD_DRONE_HEADING_ALT = 11
CMD_GIMBAL_SCAN = 21
CMD_GIMBAL_SET = 22

# hazardzonetype
ZONE_FIRE = 1
ZONE_SMOKE = 2
ZONE_NONE = 0