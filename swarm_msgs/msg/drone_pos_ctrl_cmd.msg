uint32 CTRL_CMD_IDLE_MODE = 0
uint32 CTRL_CMD_VEL_MODE  = 1
uint32 CTRL_CMD_POS_MODE  = 2
uint32 CTRL_CMD_ATT_THRUST_MODE  = 3
uint32 CTRL_CMD_ATT_VELZ_MODE  = 4

uint32 ctrl_mode

geometry_msgs/Vector3 pos_sp
geometry_msgs/Vector3 vel_sp
geometry_msgs/Quaternion att_sp
float64 yaw_sp
float64 z_sp