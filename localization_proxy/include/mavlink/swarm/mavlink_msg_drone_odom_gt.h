#pragma once
// MESSAGE DRONE_ODOM_GT PACKING

#define MAVLINK_MSG_ID_DRONE_ODOM_GT 205


typedef struct __mavlink_drone_odom_gt_t {
 int32_t lps_time; /*< [ms] LPS_TIME*/
 int16_t x; /*< [m] X Position*1000*/
 int16_t y; /*< [m] Y Position*1000*/
 int16_t z; /*< [m] Z Position*1000*/
 int16_t q0; /*< [m] QUAT W*10000*/
 int16_t q1; /*< [m] QUAT X*10000*/
 int16_t q2; /*< [m] QUAT Y*10000*/
 int16_t q3; /*< [m] QUAT Z*10000*/
 int16_t vx; /*< [m] Velocity_X*1000*/
 int16_t vy; /*< [m] Velocity_Y*1000*/
 int16_t vz; /*< [m] Velocity_Z*1000*/
 int8_t source_id; /*<  Source ID of drone*/
} mavlink_drone_odom_gt_t;

#define MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN 25
#define MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN 25
#define MAVLINK_MSG_ID_205_LEN 25
#define MAVLINK_MSG_ID_205_MIN_LEN 25

#define MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC 225
#define MAVLINK_MSG_ID_205_CRC 225



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_ODOM_GT { \
    205, \
    "DRONE_ODOM_GT", \
    12, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_odom_gt_t, lps_time) }, \
         { "source_id", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_drone_odom_gt_t, source_id) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drone_odom_gt_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drone_odom_gt_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_drone_odom_gt_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_drone_odom_gt_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_drone_odom_gt_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_drone_odom_gt_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_drone_odom_gt_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_drone_odom_gt_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_drone_odom_gt_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_drone_odom_gt_t, vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_ODOM_GT { \
    "DRONE_ODOM_GT", \
    12, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_odom_gt_t, lps_time) }, \
         { "source_id", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_drone_odom_gt_t, source_id) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drone_odom_gt_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drone_odom_gt_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_drone_odom_gt_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_drone_odom_gt_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_drone_odom_gt_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_drone_odom_gt_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_drone_odom_gt_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_drone_odom_gt_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_drone_odom_gt_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_drone_odom_gt_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_odom_gt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param q0 [m] QUAT W*10000
 * @param q1 [m] QUAT X*10000
 * @param q2 [m] QUAT Y*10000
 * @param q3 [m] QUAT Z*10000
 * @param vx [m] Velocity_X*1000
 * @param vy [m] Velocity_Y*1000
 * @param vz [m] Velocity_Z*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_odom_gt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, q0);
    _mav_put_int16_t(buf, 12, q1);
    _mav_put_int16_t(buf, 14, q2);
    _mav_put_int16_t(buf, 16, q3);
    _mav_put_int16_t(buf, 18, vx);
    _mav_put_int16_t(buf, 20, vy);
    _mav_put_int16_t(buf, 22, vz);
    _mav_put_int8_t(buf, 24, source_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN);
#else
    mavlink_drone_odom_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.source_id = source_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_ODOM_GT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
}

/**
 * @brief Pack a drone_odom_gt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param q0 [m] QUAT W*10000
 * @param q1 [m] QUAT X*10000
 * @param q2 [m] QUAT Y*10000
 * @param q3 [m] QUAT Z*10000
 * @param vx [m] Velocity_X*1000
 * @param vy [m] Velocity_Y*1000
 * @param vz [m] Velocity_Z*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_odom_gt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,int8_t source_id,int16_t x,int16_t y,int16_t z,int16_t q0,int16_t q1,int16_t q2,int16_t q3,int16_t vx,int16_t vy,int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, q0);
    _mav_put_int16_t(buf, 12, q1);
    _mav_put_int16_t(buf, 14, q2);
    _mav_put_int16_t(buf, 16, q3);
    _mav_put_int16_t(buf, 18, vx);
    _mav_put_int16_t(buf, 20, vy);
    _mav_put_int16_t(buf, 22, vz);
    _mav_put_int8_t(buf, 24, source_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN);
#else
    mavlink_drone_odom_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.source_id = source_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_ODOM_GT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
}

/**
 * @brief Encode a drone_odom_gt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_odom_gt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_odom_gt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_odom_gt_t* drone_odom_gt)
{
    return mavlink_msg_drone_odom_gt_pack(system_id, component_id, msg, drone_odom_gt->lps_time, drone_odom_gt->source_id, drone_odom_gt->x, drone_odom_gt->y, drone_odom_gt->z, drone_odom_gt->q0, drone_odom_gt->q1, drone_odom_gt->q2, drone_odom_gt->q3, drone_odom_gt->vx, drone_odom_gt->vy, drone_odom_gt->vz);
}

/**
 * @brief Encode a drone_odom_gt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_odom_gt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_odom_gt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_odom_gt_t* drone_odom_gt)
{
    return mavlink_msg_drone_odom_gt_pack_chan(system_id, component_id, chan, msg, drone_odom_gt->lps_time, drone_odom_gt->source_id, drone_odom_gt->x, drone_odom_gt->y, drone_odom_gt->z, drone_odom_gt->q0, drone_odom_gt->q1, drone_odom_gt->q2, drone_odom_gt->q3, drone_odom_gt->vx, drone_odom_gt->vy, drone_odom_gt->vz);
}

/**
 * @brief Send a drone_odom_gt message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param source_id  Source ID of drone
 * @param x [m] X Position*1000
 * @param y [m] Y Position*1000
 * @param z [m] Z Position*1000
 * @param q0 [m] QUAT W*10000
 * @param q1 [m] QUAT X*10000
 * @param q2 [m] QUAT Y*10000
 * @param q3 [m] QUAT Z*10000
 * @param vx [m] Velocity_X*1000
 * @param vy [m] Velocity_Y*1000
 * @param vz [m] Velocity_Z*1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_odom_gt_send(mavlink_channel_t chan, int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, q0);
    _mav_put_int16_t(buf, 12, q1);
    _mav_put_int16_t(buf, 14, q2);
    _mav_put_int16_t(buf, 16, q3);
    _mav_put_int16_t(buf, 18, vx);
    _mav_put_int16_t(buf, 20, vy);
    _mav_put_int16_t(buf, 22, vz);
    _mav_put_int8_t(buf, 24, source_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_ODOM_GT, buf, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
#else
    mavlink_drone_odom_gt_t packet;
    packet.lps_time = lps_time;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.source_id = source_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_ODOM_GT, (const char *)&packet, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
#endif
}

/**
 * @brief Send a drone_odom_gt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_odom_gt_send_struct(mavlink_channel_t chan, const mavlink_drone_odom_gt_t* drone_odom_gt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_odom_gt_send(chan, drone_odom_gt->lps_time, drone_odom_gt->source_id, drone_odom_gt->x, drone_odom_gt->y, drone_odom_gt->z, drone_odom_gt->q0, drone_odom_gt->q1, drone_odom_gt->q2, drone_odom_gt->q3, drone_odom_gt->vx, drone_odom_gt->vy, drone_odom_gt->vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_ODOM_GT, (const char *)drone_odom_gt, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_odom_gt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, int8_t source_id, int16_t x, int16_t y, int16_t z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, x);
    _mav_put_int16_t(buf, 6, y);
    _mav_put_int16_t(buf, 8, z);
    _mav_put_int16_t(buf, 10, q0);
    _mav_put_int16_t(buf, 12, q1);
    _mav_put_int16_t(buf, 14, q2);
    _mav_put_int16_t(buf, 16, q3);
    _mav_put_int16_t(buf, 18, vx);
    _mav_put_int16_t(buf, 20, vy);
    _mav_put_int16_t(buf, 22, vz);
    _mav_put_int8_t(buf, 24, source_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_ODOM_GT, buf, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
#else
    mavlink_drone_odom_gt_t *packet = (mavlink_drone_odom_gt_t *)msgbuf;
    packet->lps_time = lps_time;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->q0 = q0;
    packet->q1 = q1;
    packet->q2 = q2;
    packet->q3 = q3;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->source_id = source_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_ODOM_GT, (const char *)packet, MAVLINK_MSG_ID_DRONE_ODOM_GT_MIN_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN, MAVLINK_MSG_ID_DRONE_ODOM_GT_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_ODOM_GT UNPACKING


/**
 * @brief Get field lps_time from drone_odom_gt message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_drone_odom_gt_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field source_id from drone_odom_gt message
 *
 * @return  Source ID of drone
 */
static inline int8_t mavlink_msg_drone_odom_gt_get_source_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  24);
}

/**
 * @brief Get field x from drone_odom_gt message
 *
 * @return [m] X Position*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field y from drone_odom_gt message
 *
 * @return [m] Y Position*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field z from drone_odom_gt message
 *
 * @return [m] Z Position*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field q0 from drone_odom_gt message
 *
 * @return [m] QUAT W*10000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_q0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field q1 from drone_odom_gt message
 *
 * @return [m] QUAT X*10000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_q1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field q2 from drone_odom_gt message
 *
 * @return [m] QUAT Y*10000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_q2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field q3 from drone_odom_gt message
 *
 * @return [m] QUAT Z*10000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_q3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field vx from drone_odom_gt message
 *
 * @return [m] Velocity_X*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field vy from drone_odom_gt message
 *
 * @return [m] Velocity_Y*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vz from drone_odom_gt message
 *
 * @return [m] Velocity_Z*1000
 */
static inline int16_t mavlink_msg_drone_odom_gt_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Decode a drone_odom_gt message into a struct
 *
 * @param msg The message to decode
 * @param drone_odom_gt C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_odom_gt_decode(const mavlink_message_t* msg, mavlink_drone_odom_gt_t* drone_odom_gt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_odom_gt->lps_time = mavlink_msg_drone_odom_gt_get_lps_time(msg);
    drone_odom_gt->x = mavlink_msg_drone_odom_gt_get_x(msg);
    drone_odom_gt->y = mavlink_msg_drone_odom_gt_get_y(msg);
    drone_odom_gt->z = mavlink_msg_drone_odom_gt_get_z(msg);
    drone_odom_gt->q0 = mavlink_msg_drone_odom_gt_get_q0(msg);
    drone_odom_gt->q1 = mavlink_msg_drone_odom_gt_get_q1(msg);
    drone_odom_gt->q2 = mavlink_msg_drone_odom_gt_get_q2(msg);
    drone_odom_gt->q3 = mavlink_msg_drone_odom_gt_get_q3(msg);
    drone_odom_gt->vx = mavlink_msg_drone_odom_gt_get_vx(msg);
    drone_odom_gt->vy = mavlink_msg_drone_odom_gt_get_vy(msg);
    drone_odom_gt->vz = mavlink_msg_drone_odom_gt_get_vz(msg);
    drone_odom_gt->source_id = mavlink_msg_drone_odom_gt_get_source_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN? msg->len : MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN;
        memset(drone_odom_gt, 0, MAVLINK_MSG_ID_DRONE_ODOM_GT_LEN);
    memcpy(drone_odom_gt, _MAV_PAYLOAD(msg), len);
#endif
}
