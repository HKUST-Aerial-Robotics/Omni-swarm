#pragma once

#define LOOP_BOW_THRES 0.015
#define MATCH_INDEX_DIST 5
// #define MATCH_INDEX_DIST 1
#define FAST_THRES (20.0f)
#define ORB_FEATURE_SIZE (32) // For ORB
#define LOOP_FEATURE_NUM (200)
// #define USE_CUDA

// #define MIN_MOVEMENT_KEYFRAME 0.1
#define MIN_MOVEMENT_KEYFRAME 0.4

// #define LOOP_IMAGE_DOWNSAMPLE 2
extern int LOOP_IMAGE_DOWNSAMPLE;
extern int JPG_QUALITY;

#define ACCEPT_LOOP_YAW (30) //ACCEPT MAX Yaw 
#define MAX_LOOP_DIS 5.0 //ACCEPT MAX DISTANCE, 2.0 for indoor flying

extern int INIT_MODE_MIN_LOOP_NUM; //Init mode we accepte this inlier number
extern int INIT_MODE_MIN_LOOP_NUM_LEVEL2; //Init mode we accepte this inlier number
extern int MIN_LOOP_NUM;

#define MAX_LOOP_DIS_LEVEL2 3.0 //ACCEPT MAX DISTANCE, 2.0 for indoor flying

#define DEG2RAD (0.01745277777777778)

#define ACCEPT_LOOP_YAW_RAD ACCEPT_LOOP_YAW*DEG2RAD

#define USE_DEEPNET

#define DEEP_DESC_SIZE 1024

#define SEARCH_NEAREST_NUM 5
#define INNER_PRODUCT_THRES 0.75
#define ACCEPT_NONKEYFRAME_WAITSEC 5.0
#define INIT_ACCEPT_NONKEYFRAME_WAITSEC 1.0


#define INIT_MODE_PRODUCT_THRES 0.5 //INIT mode we can accept this inner product as similar

#define ORB_HAMMING_DISTANCE 40 //Max hamming
#define ORB_UV_DISTANCE 1.5 //UV distance bigger than mid*this will be removed

#define VISUALIZE_SCALE 1 //Scale for visuallize

#define CROP_WIDTH_THRES 0.05 //If movement bigger than this, crop some matches down

#define OUTLIER_XY_PRECENT_0 0.05 // This is given up match dx dy 
#define OUTLIER_XY_PRECENT_20 0.1 // This is given up match dx dy 
#define OUTLIER_XY_PRECENT_30 0.1 // This is given up match dx dy 
#define OUTLIER_XY_PRECENT_40 0.1 // This is given up match dx dy 

#define PNP_REPROJECT_ERROR 10.0
#define AVOID_GROUND_PRECENT 0.666 // This is for avoiding detect a lot feature on ground
// #define DEBUG_SHOW_IMAGE

// #define ENABLE_OPTICAL_SEC_TRY_INIT 

#define GFTT_PTS 1000
#define GFTT_MIN_DIS 10

#define USE_MATCH_MODE 
