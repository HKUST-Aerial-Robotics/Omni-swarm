#pragma once
// MESSAGE NODE_RELATIVE_FUSED PACKING

#define MAVLINK_MSG_ID_NODE_RELATIVE_FUSED 401

MAVPACKED(
typedef struct __mavlink_node_relative_fused_t {
 int32_t lps_time; /*< [ms] LPS_TIME*/
 int16_t rel_x; /*< [m] Relative X Position*1000*/
 int16_t rel_y; /*< [m] Relative Y Position*1000*/
 int16_t rel_z; /*< [m] Relative Z Position*1000*/
 int16_t rel_yaw_offset; /*< [rad] Relative Yaw coorinate offset *1000*/
 uint8_t target_id; /*<  Target ID of drone*/
}) mavlink_node_relative_fused_t;

#define MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN 13
#define MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN 13
#define MAVLINK_MSG_ID_401_LEN 13
#define MAVLINK_MSG_ID_401_MIN_LEN 13

#define MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC 164
#define MAVLINK_MSG_ID_401_CRC 164



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NODE_RELATIVE_FUSED { \
    401, \
    "NODE_RELATIVE_FUSED", \
    6, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_node_relative_fused_t, lps_time) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_node_relative_fused_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_relative_fused_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_relative_fused_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_relative_fused_t, rel_z) }, \
         { "rel_yaw_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_node_relative_fused_t, rel_yaw_offset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NODE_RELATIVE_FUSED { \
    "NODE_RELATIVE_FUSED", \
    6, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_node_relative_fused_t, lps_time) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_node_relative_fused_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_relative_fused_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_relative_fused_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_relative_fused_t, rel_z) }, \
         { "rel_yaw_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_node_relative_fused_t, rel_yaw_offset) }, \
         } \
}
#endif

/**
 * @brief Pack a node_relative_fused message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position*1000
 * @param rel_y [m] Relative Y Position*1000
 * @param rel_z [m] Relative Z Position*1000
 * @param rel_yaw_offset [rad] Relative Yaw coorinate offset *1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_relative_fused_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, uint8_t target_id, int16_t rel_x, int16_t rel_y, int16_t rel_z, int16_t rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, rel_x);
    _mav_put_int16_t(buf, 6, rel_y);
    _mav_put_int16_t(buf, 8, rel_z);
    _mav_put_int16_t(buf, 10, rel_yaw_offset);
    _mav_put_uint8_t(buf, 12, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN);
#else
    mavlink_node_relative_fused_t packet;
    packet.lps_time = lps_time;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_RELATIVE_FUSED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
}

/**
 * @brief Pack a node_relative_fused message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position*1000
 * @param rel_y [m] Relative Y Position*1000
 * @param rel_z [m] Relative Z Position*1000
 * @param rel_yaw_offset [rad] Relative Yaw coorinate offset *1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_relative_fused_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,uint8_t target_id,int16_t rel_x,int16_t rel_y,int16_t rel_z,int16_t rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, rel_x);
    _mav_put_int16_t(buf, 6, rel_y);
    _mav_put_int16_t(buf, 8, rel_z);
    _mav_put_int16_t(buf, 10, rel_yaw_offset);
    _mav_put_uint8_t(buf, 12, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN);
#else
    mavlink_node_relative_fused_t packet;
    packet.lps_time = lps_time;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_RELATIVE_FUSED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
}

/**
 * @brief Encode a node_relative_fused struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_relative_fused C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_relative_fused_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_relative_fused_t* node_relative_fused)
{
    return mavlink_msg_node_relative_fused_pack(system_id, component_id, msg, node_relative_fused->lps_time, node_relative_fused->target_id, node_relative_fused->rel_x, node_relative_fused->rel_y, node_relative_fused->rel_z, node_relative_fused->rel_yaw_offset);
}

/**
 * @brief Encode a node_relative_fused struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_relative_fused C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_relative_fused_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_relative_fused_t* node_relative_fused)
{
    return mavlink_msg_node_relative_fused_pack_chan(system_id, component_id, chan, msg, node_relative_fused->lps_time, node_relative_fused->target_id, node_relative_fused->rel_x, node_relative_fused->rel_y, node_relative_fused->rel_z, node_relative_fused->rel_yaw_offset);
}

/**
 * @brief Send a node_relative_fused message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position*1000
 * @param rel_y [m] Relative Y Position*1000
 * @param rel_z [m] Relative Z Position*1000
 * @param rel_yaw_offset [rad] Relative Yaw coorinate offset *1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_relative_fused_send(mavlink_channel_t chan, int32_t lps_time, uint8_t target_id, int16_t rel_x, int16_t rel_y, int16_t rel_z, int16_t rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, rel_x);
    _mav_put_int16_t(buf, 6, rel_y);
    _mav_put_int16_t(buf, 8, rel_z);
    _mav_put_int16_t(buf, 10, rel_yaw_offset);
    _mav_put_uint8_t(buf, 12, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED, buf, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
#else
    mavlink_node_relative_fused_t packet;
    packet.lps_time = lps_time;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED, (const char *)&packet, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
#endif
}

/**
 * @brief Send a node_relative_fused message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_node_relative_fused_send_struct(mavlink_channel_t chan, const mavlink_node_relative_fused_t* node_relative_fused)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_node_relative_fused_send(chan, node_relative_fused->lps_time, node_relative_fused->target_id, node_relative_fused->rel_x, node_relative_fused->rel_y, node_relative_fused->rel_z, node_relative_fused->rel_yaw_offset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED, (const char *)node_relative_fused, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
#endif
}

#if MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_relative_fused_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, uint8_t target_id, int16_t rel_x, int16_t rel_y, int16_t rel_z, int16_t rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, rel_x);
    _mav_put_int16_t(buf, 6, rel_y);
    _mav_put_int16_t(buf, 8, rel_z);
    _mav_put_int16_t(buf, 10, rel_yaw_offset);
    _mav_put_uint8_t(buf, 12, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED, buf, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
#else
    mavlink_node_relative_fused_t *packet = (mavlink_node_relative_fused_t *)msgbuf;
    packet->lps_time = lps_time;
    packet->rel_x = rel_x;
    packet->rel_y = rel_y;
    packet->rel_z = rel_z;
    packet->rel_yaw_offset = rel_yaw_offset;
    packet->target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED, (const char *)packet, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_CRC);
#endif
}
#endif

#endif

// MESSAGE NODE_RELATIVE_FUSED UNPACKING


/**
 * @brief Get field lps_time from node_relative_fused message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_node_relative_fused_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field target_id from node_relative_fused message
 *
 * @return  Target ID of drone
 */
static inline uint8_t mavlink_msg_node_relative_fused_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field rel_x from node_relative_fused message
 *
 * @return [m] Relative X Position*1000
 */
static inline int16_t mavlink_msg_node_relative_fused_get_rel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field rel_y from node_relative_fused message
 *
 * @return [m] Relative Y Position*1000
 */
static inline int16_t mavlink_msg_node_relative_fused_get_rel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field rel_z from node_relative_fused message
 *
 * @return [m] Relative Z Position*1000
 */
static inline int16_t mavlink_msg_node_relative_fused_get_rel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field rel_yaw_offset from node_relative_fused message
 *
 * @return [rad] Relative Yaw coorinate offset *1000
 */
static inline int16_t mavlink_msg_node_relative_fused_get_rel_yaw_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a node_relative_fused message into a struct
 *
 * @param msg The message to decode
 * @param node_relative_fused C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_relative_fused_decode(const mavlink_message_t* msg, mavlink_node_relative_fused_t* node_relative_fused)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    node_relative_fused->lps_time = mavlink_msg_node_relative_fused_get_lps_time(msg);
    node_relative_fused->rel_x = mavlink_msg_node_relative_fused_get_rel_x(msg);
    node_relative_fused->rel_y = mavlink_msg_node_relative_fused_get_rel_y(msg);
    node_relative_fused->rel_z = mavlink_msg_node_relative_fused_get_rel_z(msg);
    node_relative_fused->rel_yaw_offset = mavlink_msg_node_relative_fused_get_rel_yaw_offset(msg);
    node_relative_fused->target_id = mavlink_msg_node_relative_fused_get_target_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN? msg->len : MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN;
        memset(node_relative_fused, 0, MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_LEN);
    memcpy(node_relative_fused, _MAV_PAYLOAD(msg), len);
#endif
}
