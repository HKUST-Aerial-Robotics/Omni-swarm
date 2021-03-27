#pragma once
// MESSAGE DRONE_POSE_GT PACKING

#define MAVLINK_MSG_ID_DRONE_POSE_GT 206


typedef struct __mavlink_drone_pose_gt_t {
 int32_t lps_time; /*< [ms] LPS_TIME*/
 int16_t x; /*< [m] X Position*1000*/
 int16_t y; /*< [m] Y Position*1000*/
 int16_t z; /*< [m] Z Position*1000*/
 int16_t yaw; /*< [rad*1000] Yaw*1000*/
 int8_t source_id; /*<  Source ID of drone*/
} mavlink_drone_pose_gt_t;

#define MAVLINK_MSG_ID_DRONE_POSE_GT_LEN 13
#define MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN 13
#define MAVLINK_MSG_ID_206_LEN 13
#define MAVLINK_MSG_ID_206_MIN_LEN 13

#define MAVLINK_MSG_ID_DRONE_POSE_GT_CRC 241
#define MAVLINK_MSG_ID_206_CRC 241



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_POSE_GT { \
    206, \
    "DRONE_POSE_GT", \
    6, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_pose_gt_t, lps_time) }, \
         { "source_id", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_drone_pose_gt_t, source_id) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drone_pose_gt_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drone_pose_gt_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_drone_pose_gt_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_drone_pose_gt_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_POSE_GT { \
    "DRONE_POSE_GT", \
    6, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_pose_gt_t, lps_time) }, \
         { "source_id", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_drone_pose_gt_t, source_id) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drone_pose_gt_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drone_pose_gt_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_drone_pose_gt_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_drone_pose_gt_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_pose_gt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param yaw [rad*1000] Yaw*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_pose_gt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_POSE_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, yaw);
    _mav_put_int8_t(buf, 12, source_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN);
#else
    mavlink_drone_pose_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.source_id = source_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_POSE_GT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
}

/**
 * @brief Pack a drone_pose_gt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param yaw [rad*1000] Yaw*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_pose_gt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,int8_t source_id,int16_t x,int16_t y,int16_t z,int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_POSE_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, yaw);
    _mav_put_int8_t(buf, 12, source_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN);
#else
    mavlink_drone_pose_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.source_id = source_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_POSE_GT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
}

/**
 * @brief Encode a drone_pose_gt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_pose_gt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_pose_gt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_pose_gt_t* drone_pose_gt)
{
    return mavlink_msg_drone_pose_gt_pack(system_id, component_id, msg, drone_pose_gt->lps_time, drone_pose_gt->source_id, drone_pose_gt->x, drone_pose_gt->y, drone_pose_gt->z, drone_pose_gt->yaw);
}

/**
 * @brief Encode a drone_pose_gt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_pose_gt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_pose_gt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_pose_gt_t* drone_pose_gt)
{
    return mavlink_msg_drone_pose_gt_pack_chan(system_id, component_id, chan, msg, drone_pose_gt->lps_time, drone_pose_gt->source_id, drone_pose_gt->x, drone_pose_gt->y, drone_pose_gt->z, drone_pose_gt->yaw);
}

/**
 * @brief Send a drone_pose_gt message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param yaw [rad*1000] Yaw*1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_pose_gt_send(mavlink_channel_t chan, int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_POSE_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, yaw);
    _mav_put_int8_t(buf, 12, source_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_POSE_GT, buf, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
#else
    mavlink_drone_pose_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.source_id = source_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_POSE_GT, (const char *)&packet, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
#endif
}

/**
 * @brief Send a drone_pose_gt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_pose_gt_send_struct(mavlink_channel_t chan, const mavlink_drone_pose_gt_t* drone_pose_gt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_pose_gt_send(chan, drone_pose_gt->lps_time, drone_pose_gt->source_id, drone_pose_gt->x, drone_pose_gt->y, drone_pose_gt->z, drone_pose_gt->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_POSE_GT, (const char *)drone_pose_gt, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_POSE_GT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_pose_gt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, yaw);
    _mav_put_int8_t(buf, 12, source_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_POSE_GT, buf, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
#else
    mavlink_drone_pose_gt_t *packet = (mavlink_drone_pose_gt_t *)msgbuf;
    packet->lps_time = lps_time;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->source_id = source_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_POSE_GT, (const char *)packet, MAVLINK_MSG_ID_DRONE_POSE_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN, MAVLINK_MSG_ID_DRONE_POSE_GT_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_POSE_GT UNPACKING


/**
 * @brief Get field lps_time from drone_pose_gt message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_drone_pose_gt_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field source_id from drone_pose_gt message
 *
 * @return  Source ID of drone
 */
static inline int8_t mavlink_msg_drone_pose_gt_get_source_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  12);
}

/**
 * @brief Get field x from drone_pose_gt message
 *
 * @return [m] X Position*1000
 */
static inline int16_t mavlink_msg_drone_pose_gt_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field y from drone_pose_gt message
 *
 * @return [m] Y Position*1000
 */
static inline int16_t mavlink_msg_drone_pose_gt_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field z from drone_pose_gt message
 *
 * @return [m] Z Position*1000
 */
static inline int16_t mavlink_msg_drone_pose_gt_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field yaw from drone_pose_gt message
 *
 * @return [rad*1000] Yaw*1000
 */
static inline int16_t mavlink_msg_drone_pose_gt_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a drone_pose_gt message into a struct
 *
 * @param msg The message to decode
 * @param drone_pose_gt C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_pose_gt_decode(const mavlink_message_t* msg, mavlink_drone_pose_gt_t* drone_pose_gt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_pose_gt->lps_time = mavlink_msg_drone_pose_gt_get_lps_time(msg);
    drone_pose_gt->x = mavlink_msg_drone_pose_gt_get_x(msg);
    drone_pose_gt->y = mavlink_msg_drone_pose_gt_get_y(msg);
    drone_pose_gt->z = mavlink_msg_drone_pose_gt_get_z(msg);
    drone_pose_gt->yaw = mavlink_msg_drone_pose_gt_get_yaw(msg);
    drone_pose_gt->source_id = mavlink_msg_drone_pose_gt_get_source_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_POSE_GT_LEN? msg->len : MAVLINK_MSG_ID_DRONE_POSE_GT_LEN;
        memset(drone_pose_gt, 0, MAVLINK_MSG_ID_DRONE_POSE_GT_LEN);
    memcpy(drone_pose_gt, _MAV_PAYLOAD(msg), len);
#endif
}
