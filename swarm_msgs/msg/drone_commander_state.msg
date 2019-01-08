uint32 CTRL_INPUT_NONE=0
uint32 CTRL_INPUT_RC=1
uint32 CTRL_INPUT_ONBOARD=2

uint32 ctrl_input_state


uint32 FLIGHT_STATUS_IDLE=0 # Motor Stopped, on ground
uint32 FLIGHT_STATUS_ARMED=1 # Armed, on ground
uint32 FLIGHT_STATUS_IN_AIR=2 #Armed Inair
uint32 FLIGHT_STATUS_CRASHED=3 # Crashed, Armed

uint32 flight_status

uint32 CTRL_AUTH_RC = 0
uint32 CTRL_AUTH_APP = 1
uint32 CTRL_AUTH_THIS = 2

uint32 control_auth

float64 takeoff_target_height

uint32 CTRL_MODE_IDLE=0
uint32 CTRL_MODE_TAKEOFF=1
uint32 CTRL_MODE_LANDING=2
uint32 CTRL_MODE_HOVER=3
uint32 CTRL_MODE_POSVEL=4
uint32 CTRL_MODE_ATT=5
uint32 CTRL_MODE_MISSION=6

uint32 commander_ctrl_mode


bool is_armed
bool rc_valid
bool onboard_cmd_valid
bool vo_valid
bool djisdk_valid

swarm_msgs/drone_pos_ctrl_cmd ctrl_cmd