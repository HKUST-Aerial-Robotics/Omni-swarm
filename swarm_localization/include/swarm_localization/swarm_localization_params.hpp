#pragma once
#include <eigen3/Eigen/Dense>

extern float VO_METER_STD_TRANSLATION;
extern float VO_METER_STD_Z;
extern float VO_METER_STD_ANGLE;
extern float DISTANCE_STD;

extern float LOOP_POS_STD_0;
extern float LOOP_YAW_STD_0;
extern float LOOP_POS_STD_SLOPE;
extern float LOOP_YAW_STD_SLOPE;

extern float DETECTION_SPHERE_STD;
extern float DETECTION_INV_DEP_STD;
extern float DETECTION_DEP_STD;
extern Eigen::Vector3d CG;

#define VO_DRIFT_XYZ (Eigen::Vector3d(VO_METER_STD_TRANSLATION, VO_METER_STD_TRANSLATION, VO_METER_STD_Z))
