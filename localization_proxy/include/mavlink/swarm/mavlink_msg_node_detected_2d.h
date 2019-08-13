#pragma once
// MESSAGE NODE_DETECTED_2D PACKING

#define MAVLINK_MSG_ID_NODE_DETECTED_2D 209

MAVPACKED(
typedef struct __mavlink_node_detected_2d_t {
 int32_t lps_time; /*< [ms] LPS_TIME*/
 int16_t DY; /*< [m] Normalized DY*/
 int16_t DZ; /*< [m] Normalized DZ*/
 int16_t yaw; /*< [m] Detected Yaw*/
 int8_t target_id; /*<  Target ID of drone*/
}) mavlink_node_detected_2d_t;

#define MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN 11
#define MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN 11
#define MAVLINK_MSG_ID_209_LEN 11
#define MAVLINK_MSG_ID_209_MIN_LEN 11

#define MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC 90
#define MAVLINK_MSG_ID_209_CRC 90



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED_2D { \
    209, \
    "NODE_DETECTED_2D", \
    5, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_node_detected_2d_t, lps_time) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 10, offsetof(mavlink_node_detected_2d_t, target_id) }, \
         { "DY", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_detected_2d_t, DY) }, \
         { "DZ", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_detected_2d_t, DZ) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_detected_2d_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED_2D { \
    "NODE_DETECTED_2D", \
    5, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_node_detected_2d_t, lps_time) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 10, offsetof(mavlink_node_detected_2d_t, target_id) }, \
         { "DY", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_detected_2d_t, DY) }, \
         { "DZ", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_detected_2d_t, DZ) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_detected_2d_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a node_detected_2d message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param DY [m] Normalized DY
 * @param DZ [m] Normalized DZ
 * @param yaw [m] Detected Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_2d_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, int8_t target_id, int16_t DY, int16_t DZ, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, DY);
    _mav_put_int16_t(buf, 6, DZ);
    _mav_put_int16_t(buf, 8, yaw);
    _mav_put_int8_t(buf, 10, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN);
#else
    mavlink_node_detected_2d_t packet;
    packet.lps_time = lps_time;
    packet.DY = DY;
    packet.DZ = DZ;
    packet.yaw = yaw;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED_2D;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
}

/**
 * @brief Pack a node_detected_2d message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param DY [m] Normalized DY
 * @param DZ [m] Normalized DZ
 * @param yaw [m] Detected Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_2d_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,int8_t target_id,int16_t DY,int16_t DZ,int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, DY);
    _mav_put_int16_t(buf, 6, DZ);
    _mav_put_int16_t(buf, 8, yaw);
    _mav_put_int8_t(buf, 10, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN);
#else
    mavlink_node_detected_2d_t packet;
    packet.lps_time = lps_time;
    packet.DY = DY;
    packet.DZ = DZ;
    packet.yaw = yaw;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED_2D;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
}

/**
 * @brief Encode a node_detected_2d struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_detected_2d C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_2d_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_detected_2d_t* node_detected_2d)
{
    return mavlink_msg_node_detected_2d_pack(system_id, component_id, msg, node_detected_2d->lps_time, node_detected_2d->target_id, node_detected_2d->DY, node_detected_2d->DZ, node_detected_2d->yaw);
}

/**
 * @brief Encode a node_detected_2d struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_detected_2d C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_2d_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_detected_2d_t* node_detected_2d)
{
    return mavlink_msg_node_detected_2d_pack_chan(system_id, component_id, chan, msg, node_detected_2d->lps_time, node_detected_2d->target_id, node_detected_2d->DY, node_detected_2d->DZ, node_detected_2d->yaw);
}

/**
 * @brief Send a node_detected_2d message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param target_id  Target ID of drone
 * @param DY [m] Normalized DY
 * @param DZ [m] Normalized DZ
 * @param yaw [m] Detected Yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_detected_2d_send(mavlink_channel_t chan, int32_t lps_time, int8_t target_id, int16_t DY, int16_t DZ, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, DY);
    _mav_put_int16_t(buf, 6, DZ);
    _mav_put_int16_t(buf, 8, yaw);
    _mav_put_int8_t(buf, 10, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED_2D, buf, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
#else
    mavlink_node_detected_2d_t packet;
    packet.lps_time = lps_time;
    packet.DY = DY;
    packet.DZ = DZ;
    packet.yaw = yaw;
    packet.target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED_2D, (const char *)&packet, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
#endif
}

/**
 * @brief Send a node_detected_2d message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_node_detected_2d_send_struct(mavlink_channel_t chan, const mavlink_node_detected_2d_t* node_detected_2d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_node_detected_2d_send(chan, node_detected_2d->lps_time, node_detected_2d->target_id, node_detected_2d->DY, node_detected_2d->DZ, node_detected_2d->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED_2D, (const char *)node_detected_2d, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
#endif
}

#if MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_detected_2d_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, int8_t target_id, int16_t DY, int16_t DZ, int16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_int16_t(buf, 4, DY);
    _mav_put_int16_t(buf, 6, DZ);
    _mav_put_int16_t(buf, 8, yaw);
    _mav_put_int8_t(buf, 10, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED_2D, buf, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
#else
    mavlink_node_detected_2d_t *packet = (mavlink_node_detected_2d_t *)msgbuf;
    packet->lps_time = lps_time;
    packet->DY = DY;
    packet->DZ = DZ;
    packet->yaw = yaw;
    packet->target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED_2D, (const char *)packet, MAVLINK_MSG_ID_NODE_DETECTED_2D_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN, MAVLINK_MSG_ID_NODE_DETECTED_2D_CRC);
#endif
}
#endif

#endif

// MESSAGE NODE_DETECTED_2D UNPACKING


/**
 * @brief Get field lps_time from node_detected_2d message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_node_detected_2d_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field target_id from node_detected_2d message
 *
 * @return  Target ID of drone
 */
static inline int8_t mavlink_msg_node_detected_2d_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  10);
}

/**
 * @brief Get field DY from node_detected_2d message
 *
 * @return [m] Normalized DY
 */
static inline int16_t mavlink_msg_node_detected_2d_get_DY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field DZ from node_detected_2d message
 *
 * @return [m] Normalized DZ
 */
static inline int16_t mavlink_msg_node_detected_2d_get_DZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field yaw from node_detected_2d message
 *
 * @return [m] Detected Yaw
 */
static inline int16_t mavlink_msg_node_detected_2d_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Decode a node_detected_2d message into a struct
 *
 * @param msg The message to decode
 * @param node_detected_2d C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_detected_2d_decode(const mavlink_message_t* msg, mavlink_node_detected_2d_t* node_detected_2d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    node_detected_2d->lps_time = mavlink_msg_node_detected_2d_get_lps_time(msg);
    node_detected_2d->DY = mavlink_msg_node_detected_2d_get_DY(msg);
    node_detected_2d->DZ = mavlink_msg_node_detected_2d_get_DZ(msg);
    node_detected_2d->yaw = mavlink_msg_node_detected_2d_get_yaw(msg);
    node_detected_2d->target_id = mavlink_msg_node_detected_2d_get_target_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN? msg->len : MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN;
        memset(node_detected_2d, 0, MAVLINK_MSG_ID_NODE_DETECTED_2D_LEN);
    memcpy(node_detected_2d, _MAV_PAYLOAD(msg), len);
#endif
}
