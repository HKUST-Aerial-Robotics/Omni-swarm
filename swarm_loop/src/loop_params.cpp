#include "swarm_loop/loop_defines.h"

int JPG_QUALITY;

int INIT_MODE_MIN_LOOP_NUM; //Init mode we accepte this inlier number
int INIT_MODE_MIN_LOOP_NUM_LEVEL2;
int MIN_LOOP_NUM;
bool ENABLE_LK_LOOP_DETECTION;

double INNER_PRODUCT_THRES;
double INIT_MODE_PRODUCT_THRES;//INIT mode we can accept this inner product as similar
int MATCH_INDEX_DIST;
int ACCEPT_MIN_3D_PTS;
int MIN_MATCH_PRE_DIR;
std::string OUTPUT_PATH;
bool IS_PC_REPLAY;
bool SEND_ALL_FEATURES;
bool LOWER_CAM_AS_MAIN;
int MAX_DIRS;
bool OUTPUT_RAW_SUPERPOINT_DESC;
double DEPTH_NEAR_THRES;
double DEPTH_FAR_THRES;

int MIN_DIRECTION_LOOP;
double DETECTOR_MATCH_THRES;
double loop_cov_pos;
double loop_cov_ang;