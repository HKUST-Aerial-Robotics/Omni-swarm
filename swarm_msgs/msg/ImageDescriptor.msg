Header header
int32 drone_id

float32[] feature_descriptor

geometry_msgs/Pose pose_drone
geometry_msgs/Pose camera_extrinsic

float32[] image_desc
int32 image_width
int32 image_height
uint8[] image

geometry_msgs/Point32[] landmarks_2d_norm
geometry_msgs/Point32[] landmarks_2d
geometry_msgs/Point32[] landmarks_3d
uint8[] landmarks_flag
bool prevent_adding_db

int32 direction