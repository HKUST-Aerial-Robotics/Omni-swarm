#pragma once
// MESSAGE NODE_REALTIME_INFO PACKING

#define MAVLINK_MSG_ID_NODE_REALTIME_INFO 400

MAVPACKED(
typedef struct __mavlink_node_realtime_info_t {
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 int16_t q0; /*< [m] q0*10000*/
 int16_t q1; /*< [m] q1*10000*/
 int16_t q2; /*< [m] q2*10000*/
 int16_t q3; /*< [m] q3*10000*/
 int16_t vx; /*< [m/s] X linear speed*1000*/
 int16_t vy; /*< [m/s] Y linear speed*1000*/
 int16_t vz; /*< [m/s] Z linear speed*1000*/
 int16_t remote_distance[10]; /*< [m] Distance to Remote Drone*1000*/
 uint8_t odom_vaild; /*<  If odometry is vaild*/
}) mavlink_node_realtime_info_t;

#define MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN 47
#define MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN 47
#define MAVLINK_MSG_ID_400_LEN 47
#define MAVLINK_MSG_ID_400_MIN_LEN 47

#define MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC 62
#define MAVLINK_MSG_ID_400_CRC 62

#define MAVLINK_MSG_NODE_REALTIME_INFO_FIELD_REMOTE_DISTANCE_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NODE_REALTIME_INFO { \
    400, \
    "NODE_REALTIME_INFO", \
    12, \
    {  { "odom_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_node_realtime_info_t, odom_vaild) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_node_realtime_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_node_realtime_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_node_realtime_info_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_node_realtime_info_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_node_realtime_info_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_node_realtime_info_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_node_realtime_info_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_node_realtime_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_node_realtime_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_node_realtime_info_t, vz) }, \
         { "remote_distance", NULL, MAVLINK_TYPE_INT16_T, 10, 26, offsetof(mavlink_node_realtime_info_t, remote_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NODE_REALTIME_INFO { \
    "NODE_REALTIME_INFO", \
    12, \
    {  { "odom_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_node_realtime_info_t, odom_vaild) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_node_realtime_info_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_node_realtime_info_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_node_realtime_info_t, z) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_node_realtime_info_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_node_realtime_info_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_node_realtime_info_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_node_realtime_info_t, q3) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_node_realtime_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_node_realtime_info_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_node_realtime_info_t, vz) }, \
         { "remote_distance", NULL, MAVLINK_TYPE_INT16_T, 10, 26, offsetof(mavlink_node_realtime_info_t, remote_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a node_realtime_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0*10000
 * @param q1 [m] q1*10000
 * @param q2 [m] q2*10000
 * @param q3 [m] q3*10000
 * @param vx [m/s] X linear speed*1000
 * @param vy [m/s] Y linear speed*1000
 * @param vz [m/s] Z linear speed*1000
 * @param remote_distance [m] Distance to Remote Drone*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_realtime_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t odom_vaild, float x, float y, float z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz, const int16_t *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_int16_t(buf, 12, q0);
    _mav_put_int16_t(buf, 14, q1);
    _mav_put_int16_t(buf, 16, q2);
    _mav_put_int16_t(buf, 18, q3);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint8_t(buf, 46, odom_vaild);
    _mav_put_int16_t_array(buf, 26, remote_distance, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN);
#else
    mavlink_node_realtime_info_t packet;
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
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(int16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_REALTIME_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
}

/**
 * @brief Pack a node_realtime_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0*10000
 * @param q1 [m] q1*10000
 * @param q2 [m] q2*10000
 * @param q3 [m] q3*10000
 * @param vx [m/s] X linear speed*1000
 * @param vy [m/s] Y linear speed*1000
 * @param vz [m/s] Z linear speed*1000
 * @param remote_distance [m] Distance to Remote Drone*1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_realtime_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t odom_vaild,float x,float y,float z,int16_t q0,int16_t q1,int16_t q2,int16_t q3,int16_t vx,int16_t vy,int16_t vz,const int16_t *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_int16_t(buf, 12, q0);
    _mav_put_int16_t(buf, 14, q1);
    _mav_put_int16_t(buf, 16, q2);
    _mav_put_int16_t(buf, 18, q3);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint8_t(buf, 46, odom_vaild);
    _mav_put_int16_t_array(buf, 26, remote_distance, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN);
#else
    mavlink_node_realtime_info_t packet;
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
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(int16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_REALTIME_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
}

/**
 * @brief Encode a node_realtime_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_realtime_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_realtime_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_realtime_info_t* node_realtime_info)
{
    return mavlink_msg_node_realtime_info_pack(system_id, component_id, msg, node_realtime_info->odom_vaild, node_realtime_info->x, node_realtime_info->y, node_realtime_info->z, node_realtime_info->q0, node_realtime_info->q1, node_realtime_info->q2, node_realtime_info->q3, node_realtime_info->vx, node_realtime_info->vy, node_realtime_info->vz, node_realtime_info->remote_distance);
}

/**
 * @brief Encode a node_realtime_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_realtime_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_realtime_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_realtime_info_t* node_realtime_info)
{
    return mavlink_msg_node_realtime_info_pack_chan(system_id, component_id, chan, msg, node_realtime_info->odom_vaild, node_realtime_info->x, node_realtime_info->y, node_realtime_info->z, node_realtime_info->q0, node_realtime_info->q1, node_realtime_info->q2, node_realtime_info->q3, node_realtime_info->vx, node_realtime_info->vy, node_realtime_info->vz, node_realtime_info->remote_distance);
}

/**
 * @brief Send a node_realtime_info message
 * @param chan MAVLink channel to send the message
 *
 * @param odom_vaild  If odometry is vaild
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q0 [m] q0*10000
 * @param q1 [m] q1*10000
 * @param q2 [m] q2*10000
 * @param q3 [m] q3*10000
 * @param vx [m/s] X linear speed*1000
 * @param vy [m/s] Y linear speed*1000
 * @param vz [m/s] Z linear speed*1000
 * @param remote_distance [m] Distance to Remote Drone*1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_realtime_info_send(mavlink_channel_t chan, uint8_t odom_vaild, float x, float y, float z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz, const int16_t *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_int16_t(buf, 12, q0);
    _mav_put_int16_t(buf, 14, q1);
    _mav_put_int16_t(buf, 16, q2);
    _mav_put_int16_t(buf, 18, q3);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint8_t(buf, 46, odom_vaild);
    _mav_put_int16_t_array(buf, 26, remote_distance, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO, buf, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
#else
    mavlink_node_realtime_info_t packet;
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
    mav_array_memcpy(packet.remote_distance, remote_distance, sizeof(int16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO, (const char *)&packet, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
#endif
}

/**
 * @brief Send a node_realtime_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_node_realtime_info_send_struct(mavlink_channel_t chan, const mavlink_node_realtime_info_t* node_realtime_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_node_realtime_info_send(chan, node_realtime_info->odom_vaild, node_realtime_info->x, node_realtime_info->y, node_realtime_info->z, node_realtime_info->q0, node_realtime_info->q1, node_realtime_info->q2, node_realtime_info->q3, node_realtime_info->vx, node_realtime_info->vy, node_realtime_info->vz, node_realtime_info->remote_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO, (const char *)node_realtime_info, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_realtime_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t odom_vaild, float x, float y, float z, int16_t q0, int16_t q1, int16_t q2, int16_t q3, int16_t vx, int16_t vy, int16_t vz, const int16_t *remote_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_int16_t(buf, 12, q0);
    _mav_put_int16_t(buf, 14, q1);
    _mav_put_int16_t(buf, 16, q2);
    _mav_put_int16_t(buf, 18, q3);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint8_t(buf, 46, odom_vaild);
    _mav_put_int16_t_array(buf, 26, remote_distance, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO, buf, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
#else
    mavlink_node_realtime_info_t *packet = (mavlink_node_realtime_info_t *)msgbuf;
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
    mav_array_memcpy(packet->remote_distance, remote_distance, sizeof(int16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_REALTIME_INFO, (const char *)packet, MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN, MAVLINK_MSG_ID_NODE_REALTIME_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE NODE_REALTIME_INFO UNPACKING


/**
 * @brief Get field odom_vaild from node_realtime_info message
 *
 * @return  If odometry is vaild
 */
static inline uint8_t mavlink_msg_node_realtime_info_get_odom_vaild(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field x from node_realtime_info message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_node_realtime_info_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from node_realtime_info message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_node_realtime_info_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from node_realtime_info message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_node_realtime_info_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field q0 from node_realtime_info message
 *
 * @return [m] q0*10000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_q0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field q1 from node_realtime_info message
 *
 * @return [m] q1*10000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_q1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field q2 from node_realtime_info message
 *
 * @return [m] q2*10000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_q2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field q3 from node_realtime_info message
 *
 * @return [m] q3*10000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_q3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field vx from node_realtime_info message
 *
 * @return [m/s] X linear speed*1000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from node_realtime_info message
 *
 * @return [m/s] Y linear speed*1000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from node_realtime_info message
 *
 * @return [m/s] Z linear speed*1000
 */
static inline int16_t mavlink_msg_node_realtime_info_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field remote_distance from node_realtime_info message
 *
 * @return [m] Distance to Remote Drone*1000
 */
static inline uint16_t mavlink_msg_node_realtime_info_get_remote_distance(const mavlink_message_t* msg, int16_t *remote_distance)
{
    return _MAV_RETURN_int16_t_array(msg, remote_distance, 10,  26);
}

/**
 * @brief Decode a node_realtime_info message into a struct
 *
 * @param msg The message to decode
 * @param node_realtime_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_realtime_info_decode(const mavlink_message_t* msg, mavlink_node_realtime_info_t* node_realtime_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    node_realtime_info->x = mavlink_msg_node_realtime_info_get_x(msg);
    node_realtime_info->y = mavlink_msg_node_realtime_info_get_y(msg);
    node_realtime_info->z = mavlink_msg_node_realtime_info_get_z(msg);
    node_realtime_info->q0 = mavlink_msg_node_realtime_info_get_q0(msg);
    node_realtime_info->q1 = mavlink_msg_node_realtime_info_get_q1(msg);
    node_realtime_info->q2 = mavlink_msg_node_realtime_info_get_q2(msg);
    node_realtime_info->q3 = mavlink_msg_node_realtime_info_get_q3(msg);
    node_realtime_info->vx = mavlink_msg_node_realtime_info_get_vx(msg);
    node_realtime_info->vy = mavlink_msg_node_realtime_info_get_vy(msg);
    node_realtime_info->vz = mavlink_msg_node_realtime_info_get_vz(msg);
    mavlink_msg_node_realtime_info_get_remote_distance(msg, node_realtime_info->remote_distance);
    node_realtime_info->odom_vaild = mavlink_msg_node_realtime_info_get_odom_vaild(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN? msg->len : MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN;
        memset(node_realtime_info, 0, MAVLINK_MSG_ID_NODE_REALTIME_INFO_LEN);
    memcpy(node_realtime_info, _MAV_PAYLOAD(msg), len);
#endif
}
