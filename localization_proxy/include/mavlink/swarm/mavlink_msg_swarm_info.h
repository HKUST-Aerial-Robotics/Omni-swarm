#pragma once
// MESSAGE SWARM_INFO PACKING

#define MAVLINK_MSG_ID_SWARM_INFO 400

MAVPACKED(
typedef struct __mavlink_swarm_info_t {
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float q0; /*< [m] q0*/
 float q1; /*< [m] q1*/
 float q2; /*< [m] q2*/
 float q3; /*< [m] q3*/
 float vx; /*< [m/s] X linear speed*/
 float vy; /*< [m/s] Y linear speed*/
 float vz; /*< [m/s] Z linear speed*/
 float remote_distance[10]; /*< [m] Distance to Remote Drone*/
 uint8_t odom_vaild; /*<  If odometry is vaild*/
}) mavlink_swarm_info_t;

#define MAVLINK_MSG_ID_SWARM_INFO_LEN 81
#define MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN 81
#define MAVLINK_MSG_ID_400_LEN 81
#define MAVLINK_MSG_ID_400_MIN_LEN 81

#define MAVLINK_MSG_ID_SWARM_INFO_CRC 59
#define MAVLINK_MSG_ID_400_CRC 59

#define MAVLINK_MSG_SWARM_INFO_FIELD_REMOTE_DISTANCE_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SWARM_INFO { \
    400, \
    "SWARM_INFO", \
    12, \
    {  { "odom_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_swarm_info_t, odom_vaild) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_swarm_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_swarm_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_swarm_info_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_swarm_info_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_swarm_info_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_swarm_info_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_swarm_info_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_swarm_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_swarm_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_swarm_info_t, vz) }, \
         { "remote_distance", NULL, MAVLINK_TYPE_FLOAT, 10, 40, offsetof(mavlink_swarm_info_t, remote_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SWARM_INFO { \
    "SWARM_INFO", \
    12, \
    {  { "odom_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_swarm_info_t, odom_vaild) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_swarm_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_swarm_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_swarm_info_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_swarm_info_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_swarm_info_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_swarm_info_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_swarm_info_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_swarm_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_swarm_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_swarm_info_t, vz) }, \
         { "remote_distance", NULL, MAVLINK_TYPE_FLOAT, 10, 40, offsetof(mavlink_swarm_info_t, remote_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a swarm_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0
 * @param q1 [m] q1
 * @param q2 [m] q2
 * @param q3 [m] q3
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param remote_distance [m] Distance to Remote Drone
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t odom_vaild, float x, float y, float z, float q0, float q1, float q2, float q3, float vx, float vy, float vz, const float *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, q0);
    _mav_put_float(buf, 16, q1);
    _mav_put_float(buf, 20, q2);
    _mav_put_float(buf, 24, q3);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint8_t(buf, 80, odom_vaild);
    _mav_put_float_array(buf, 40, remote_distance, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_INFO_LEN);
#else
    mavlink_swarm_info_t packet;
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
    packet.odom_vaild = odom_vaild;
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
}

/**
 * @brief Pack a swarm_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0
 * @param q1 [m] q1
 * @param q2 [m] q2
 * @param q3 [m] q3
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param remote_distance [m] Distance to Remote Drone
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t odom_vaild,float x,float y,float z,float q0,float q1,float q2,float q3,float vx,float vy,float vz,const float *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, q0);
    _mav_put_float(buf, 16, q1);
    _mav_put_float(buf, 20, q2);
    _mav_put_float(buf, 24, q3);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint8_t(buf, 80, odom_vaild);
    _mav_put_float_array(buf, 40, remote_distance, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_INFO_LEN);
#else
    mavlink_swarm_info_t packet;
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
    packet.odom_vaild = odom_vaild;
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
}

/**
 * @brief Encode a swarm_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param swarm_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_swarm_info_t* swarm_info)
{
    return mavlink_msg_swarm_info_pack(system_id, component_id, msg, swarm_info->odom_vaild, swarm_info->x, swarm_info->y, swarm_info->z, swarm_info->q0, swarm_info->q1, swarm_info->q2, swarm_info->q3, swarm_info->vx, swarm_info->vy, swarm_info->vz, swarm_info->remote_distance);
}

/**
 * @brief Encode a swarm_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_swarm_info_t* swarm_info)
{
    return mavlink_msg_swarm_info_pack_chan(system_id, component_id, chan, msg, swarm_info->odom_vaild, swarm_info->x, swarm_info->y, swarm_info->z, swarm_info->q0, swarm_info->q1, swarm_info->q2, swarm_info->q3, swarm_info->vx, swarm_info->vy, swarm_info->vz, swarm_info->remote_distance);
}

/**
 * @brief Send a swarm_info message
 * @param chan MAVLink channel to send the message
 *
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0
 * @param q1 [m] q1
 * @param q2 [m] q2
 * @param q3 [m] q3
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param remote_distance [m] Distance to Remote Drone
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_swarm_info_send(mavlink_channel_t chan, uint8_t odom_vaild, float x, float y, float z, float q0, float q1, float q2, float q3, float vx, float vy, float vz, const float *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, q0);
    _mav_put_float(buf, 16, q1);
    _mav_put_float(buf, 20, q2);
    _mav_put_float(buf, 24, q3);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint8_t(buf, 80, odom_vaild);
    _mav_put_float_array(buf, 40, remote_distance, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_INFO, buf, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
#else
    mavlink_swarm_info_t packet;
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
    packet.odom_vaild = odom_vaild;
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_INFO, (const char *)&packet, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
#endif
}

/**
 * @brief Send a swarm_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_swarm_info_send_struct(mavlink_channel_t chan, const mavlink_swarm_info_t* swarm_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_swarm_info_send(chan, swarm_info->odom_vaild, swarm_info->x, swarm_info->y, swarm_info->z, swarm_info->q0, swarm_info->q1, swarm_info->q2, swarm_info->q3, swarm_info->vx, swarm_info->vy, swarm_info->vz, swarm_info->remote_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_INFO, (const char *)swarm_info, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_SWARM_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_swarm_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t odom_vaild, float x, float y, float z, float q0, float q1, float q2, float q3, float vx, float vy, float vz, const float *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, q0);
    _mav_put_float(buf, 16, q1);
    _mav_put_float(buf, 20, q2);
    _mav_put_float(buf, 24, q3);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint8_t(buf, 80, odom_vaild);
    _mav_put_float_array(buf, 40, remote_distance, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_INFO, buf, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
#else
    mavlink_swarm_info_t *packet = (mavlink_swarm_info_t *)msgbuf;
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
    packet->odom_vaild = odom_vaild;
    mav_array_memcpy(packet->remote_distance, remote_distance, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_INFO, (const char *)packet, MAVLINK_MSG_ID_SWARM_INFO_MIN_LEN, MAVLINK_MSG_ID_SWARM_INFO_LEN, MAVLINK_MSG_ID_SWARM_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE SWARM_INFO UNPACKING


/**
 * @brief Get field odom_vaild from swarm_info message
 *
 * @return  If odometry is vaild
 */
static inline uint8_t mavlink_msg_swarm_info_get_odom_vaild(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field x from swarm_info message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_swarm_info_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from swarm_info message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_swarm_info_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from swarm_info message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_swarm_info_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field q0 from swarm_info message
 *
 * @return [m] q0
 */
static inline float mavlink_msg_swarm_info_get_q0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field q1 from swarm_info message
 *
 * @return [m] q1
 */
static inline float mavlink_msg_swarm_info_get_q1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field q2 from swarm_info message
 *
 * @return [m] q2
 */
static inline float mavlink_msg_swarm_info_get_q2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field q3 from swarm_info message
 *
 * @return [m] q3
 */
static inline float mavlink_msg_swarm_info_get_q3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vx from swarm_info message
 *
 * @return [m/s] X linear speed
 */
static inline float mavlink_msg_swarm_info_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vy from swarm_info message
 *
 * @return [m/s] Y linear speed
 */
static inline float mavlink_msg_swarm_info_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vz from swarm_info message
 *
 * @return [m/s] Z linear speed
 */
static inline float mavlink_msg_swarm_info_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field remote_distance from swarm_info message
 *
 * @return [m] Distance to Remote Drone
 */
static inline uint16_t mavlink_msg_swarm_info_get_remote_distance(const mavlink_message_t* msg, float *remote_distance)
{
    return _MAV_RETURN_float_array(msg, remote_distance, 10,  40);
}

/**
 * @brief Decode a swarm_info message into a struct
 *
 * @param msg The message to decode
 * @param swarm_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_swarm_info_decode(const mavlink_message_t* msg, mavlink_swarm_info_t* swarm_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    swarm_info->x = mavlink_msg_swarm_info_get_x(msg);
    swarm_info->y = mavlink_msg_swarm_info_get_y(msg);
    swarm_info->z = mavlink_msg_swarm_info_get_z(msg);
    swarm_info->q0 = mavlink_msg_swarm_info_get_q0(msg);
    swarm_info->q1 = mavlink_msg_swarm_info_get_q1(msg);
    swarm_info->q2 = mavlink_msg_swarm_info_get_q2(msg);
    swarm_info->q3 = mavlink_msg_swarm_info_get_q3(msg);
    swarm_info->vx = mavlink_msg_swarm_info_get_vx(msg);
    swarm_info->vy = mavlink_msg_swarm_info_get_vy(msg);
    swarm_info->vz = mavlink_msg_swarm_info_get_vz(msg);
    mavlink_msg_swarm_info_get_remote_distance(msg, swarm_info->remote_distance);
    swarm_info->odom_vaild = mavlink_msg_swarm_info_get_odom_vaild(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SWARM_INFO_LEN? msg->len : MAVLINK_MSG_ID_SWARM_INFO_LEN;
        memset(swarm_info, 0, MAVLINK_MSG_ID_SWARM_INFO_LEN);
    memcpy(swarm_info, _MAV_PAYLOAD(msg), len);
#endif
}
