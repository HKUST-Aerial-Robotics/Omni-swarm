#pragma once
// MESSAGE SWARM_RELATIVE_FUSED PACKING

#define MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED 401

MAVPACKED(
typedef struct __mavlink_swarm_relative_fused_t {
 float rel_x; /*< [m] Relative X Position*/
 float rel_y; /*< [m] Relative Y Position*/
 float rel_z; /*< [m] Relative Z Position*/
 float rel_yaw_offset; /*< [deg] Relative Yaw coorinate offset*/
 uint8_t source_id; /*<  Source ID of drone*/
 uint8_t target_id; /*<  Target ID of drone*/
}) mavlink_swarm_relative_fused_t;

#define MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN 18
#define MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN 18
#define MAVLINK_MSG_ID_401_LEN 18
#define MAVLINK_MSG_ID_401_MIN_LEN 18

#define MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC 63
#define MAVLINK_MSG_ID_401_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SWARM_RELATIVE_FUSED { \
    401, \
    "SWARM_RELATIVE_FUSED", \
    6, \
    {  { "source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_swarm_relative_fused_t, source_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_swarm_relative_fused_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_swarm_relative_fused_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_swarm_relative_fused_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_swarm_relative_fused_t, rel_z) }, \
         { "rel_yaw_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_swarm_relative_fused_t, rel_yaw_offset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SWARM_RELATIVE_FUSED { \
    "SWARM_RELATIVE_FUSED", \
    6, \
    {  { "source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_swarm_relative_fused_t, source_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_swarm_relative_fused_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_swarm_relative_fused_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_swarm_relative_fused_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_swarm_relative_fused_t, rel_z) }, \
         { "rel_yaw_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_swarm_relative_fused_t, rel_yaw_offset) }, \
         } \
}
#endif

/**
 * @brief Pack a swarm_relative_fused message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw_offset [deg] Relative Yaw coorinate offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_relative_fused_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t source_id, uint8_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN];
    _mav_put_float(buf, 0, rel_x);
    _mav_put_float(buf, 4, rel_y);
    _mav_put_float(buf, 8, rel_z);
    _mav_put_float(buf, 12, rel_yaw_offset);
    _mav_put_uint8_t(buf, 16, source_id);
    _mav_put_uint8_t(buf, 17, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN);
#else
    mavlink_swarm_relative_fused_t packet;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.source_id = source_id;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
}

/**
 * @brief Pack a swarm_relative_fused message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw_offset [deg] Relative Yaw coorinate offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_relative_fused_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t source_id,uint8_t target_id,float rel_x,float rel_y,float rel_z,float rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN];
    _mav_put_float(buf, 0, rel_x);
    _mav_put_float(buf, 4, rel_y);
    _mav_put_float(buf, 8, rel_z);
    _mav_put_float(buf, 12, rel_yaw_offset);
    _mav_put_uint8_t(buf, 16, source_id);
    _mav_put_uint8_t(buf, 17, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN);
#else
    mavlink_swarm_relative_fused_t packet;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.source_id = source_id;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
}

/**
 * @brief Encode a swarm_relative_fused struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param swarm_relative_fused C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_relative_fused_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_swarm_relative_fused_t* swarm_relative_fused)
{
    return mavlink_msg_swarm_relative_fused_pack(system_id, component_id, msg, swarm_relative_fused->source_id, swarm_relative_fused->target_id, swarm_relative_fused->rel_x, swarm_relative_fused->rel_y, swarm_relative_fused->rel_z, swarm_relative_fused->rel_yaw_offset);
}

/**
 * @brief Encode a swarm_relative_fused struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_relative_fused C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_relative_fused_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_swarm_relative_fused_t* swarm_relative_fused)
{
    return mavlink_msg_swarm_relative_fused_pack_chan(system_id, component_id, chan, msg, swarm_relative_fused->source_id, swarm_relative_fused->target_id, swarm_relative_fused->rel_x, swarm_relative_fused->rel_y, swarm_relative_fused->rel_z, swarm_relative_fused->rel_yaw_offset);
}

/**
 * @brief Send a swarm_relative_fused message
 * @param chan MAVLink channel to send the message
 *
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw_offset [deg] Relative Yaw coorinate offset
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_swarm_relative_fused_send(mavlink_channel_t chan, uint8_t source_id, uint8_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN];
    _mav_put_float(buf, 0, rel_x);
    _mav_put_float(buf, 4, rel_y);
    _mav_put_float(buf, 8, rel_z);
    _mav_put_float(buf, 12, rel_yaw_offset);
    _mav_put_uint8_t(buf, 16, source_id);
    _mav_put_uint8_t(buf, 17, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED, buf, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
#else
    mavlink_swarm_relative_fused_t packet;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw_offset = rel_yaw_offset;
    packet.source_id = source_id;
    packet.target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED, (const char *)&packet, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
#endif
}

/**
 * @brief Send a swarm_relative_fused message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_swarm_relative_fused_send_struct(mavlink_channel_t chan, const mavlink_swarm_relative_fused_t* swarm_relative_fused)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_swarm_relative_fused_send(chan, swarm_relative_fused->source_id, swarm_relative_fused->target_id, swarm_relative_fused->rel_x, swarm_relative_fused->rel_y, swarm_relative_fused->rel_z, swarm_relative_fused->rel_yaw_offset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED, (const char *)swarm_relative_fused, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
#endif
}

#if MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_swarm_relative_fused_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t source_id, uint8_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rel_x);
    _mav_put_float(buf, 4, rel_y);
    _mav_put_float(buf, 8, rel_z);
    _mav_put_float(buf, 12, rel_yaw_offset);
    _mav_put_uint8_t(buf, 16, source_id);
    _mav_put_uint8_t(buf, 17, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED, buf, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
#else
    mavlink_swarm_relative_fused_t *packet = (mavlink_swarm_relative_fused_t *)msgbuf;
    packet->rel_x = rel_x;
    packet->rel_y = rel_y;
    packet->rel_z = rel_z;
    packet->rel_yaw_offset = rel_yaw_offset;
    packet->source_id = source_id;
    packet->target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED, (const char *)packet, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_MIN_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_CRC);
#endif
}
#endif

#endif

// MESSAGE SWARM_RELATIVE_FUSED UNPACKING


/**
 * @brief Get field source_id from swarm_relative_fused message
 *
 * @return  Source ID of drone
 */
static inline uint8_t mavlink_msg_swarm_relative_fused_get_source_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_id from swarm_relative_fused message
 *
 * @return  Target ID of drone
 */
static inline uint8_t mavlink_msg_swarm_relative_fused_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field rel_x from swarm_relative_fused message
 *
 * @return [m] Relative X Position
 */
static inline float mavlink_msg_swarm_relative_fused_get_rel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rel_y from swarm_relative_fused message
 *
 * @return [m] Relative Y Position
 */
static inline float mavlink_msg_swarm_relative_fused_get_rel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rel_z from swarm_relative_fused message
 *
 * @return [m] Relative Z Position
 */
static inline float mavlink_msg_swarm_relative_fused_get_rel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rel_yaw_offset from swarm_relative_fused message
 *
 * @return [deg] Relative Yaw coorinate offset
 */
static inline float mavlink_msg_swarm_relative_fused_get_rel_yaw_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a swarm_relative_fused message into a struct
 *
 * @param msg The message to decode
 * @param swarm_relative_fused C-struct to decode the message contents into
 */
static inline void mavlink_msg_swarm_relative_fused_decode(const mavlink_message_t* msg, mavlink_swarm_relative_fused_t* swarm_relative_fused)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    swarm_relative_fused->rel_x = mavlink_msg_swarm_relative_fused_get_rel_x(msg);
    swarm_relative_fused->rel_y = mavlink_msg_swarm_relative_fused_get_rel_y(msg);
    swarm_relative_fused->rel_z = mavlink_msg_swarm_relative_fused_get_rel_z(msg);
    swarm_relative_fused->rel_yaw_offset = mavlink_msg_swarm_relative_fused_get_rel_yaw_offset(msg);
    swarm_relative_fused->source_id = mavlink_msg_swarm_relative_fused_get_source_id(msg);
    swarm_relative_fused->target_id = mavlink_msg_swarm_relative_fused_get_target_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN? msg->len : MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN;
        memset(swarm_relative_fused, 0, MAVLINK_MSG_ID_SWARM_RELATIVE_FUSED_LEN);
    memcpy(swarm_relative_fused, _MAV_PAYLOAD(msg), len);
#endif
}
