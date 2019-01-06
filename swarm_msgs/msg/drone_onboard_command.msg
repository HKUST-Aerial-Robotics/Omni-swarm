uint32 CTRL_POS_COMMAND=0
uint32 CTRL_VEL_COMMAND=1
uint32 CTRL_ATT_COMMAND=2
uint32 CTRL_MISSION_LOAD_COMMAND=3
uint32 CTRL_MISSION_END_COMMAND=4
uint32 CTRL_TAKEOF_COMMAND=5
uint32 CTRL_LAND_COMMAND=6
uint32 CTRL_HOVER_COMMAND=7
uint32 CTRL_ARM_COMMAND=8
uint32 CTRL_DISARM_COMMAND=9

uint32 command_type

# For POS/VEL param1 2 3 is x y z
# For ATT param1 2 3 is roll pitch yaw vz
uint32 param1
uint32 param2
uint32 param3
uint32 param4
uint32 param5
uint32 param6
uint32 param7