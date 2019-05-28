#pragma once
// MESSAGE NODE_DETECTED PACKING

#define MAVLINK_MSG_ID_NODE_DETECTED 403

MAVPACKED(
typedef struct __mavlink_node_detected_t {
 int16_t ts; /*< [ms] Timestamp*/
 int16_t x; /*< [m] Relative X Position*1000*/
 int16_t y; /*< [m] Relative Y Position*1000*/
 int16_t z; /*< [m] Relative Z Position*1000*/
 int16_t corx; /*< [m] Relative X Position Corvariance*1000*/
 int16_t cory; /*< [m] Relative X Position Corvariance*1000*/
 int16_t corz; /*< [m] Relative X Position Corvariance*1000*/
 int16_t q0; /*< [m] Relative QUAT W*10000*/
 int16_t q1; /*< [m] Relative QUAT X*10000*/
 int16_t q2; /*< [m] Relative QUAT Y*10000*/
 int16_t q3; /*< [m] Relative QUAT Z*10000*/
 int8_t target_id; /*<  Target ID of drone*/
}) mavlink_node_detected_t;

#define MAVLINK_MSG_ID_NODE_DETECTED_LEN 23
#define MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN 23
#define MAVLINK_MSG_ID_403_LEN 23
#define MAVLINK_MSG_ID_403_MIN_LEN 23

#define MAVLINK_MSG_ID_NODE_DETECTED_CRC 227
#define MAVLINK_MSG_ID_403_CRC 227



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED { \
    403, \
    "NODE_DETECTED", \
    12, \
    {  { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 22, offsetof(mavlink_node_detected_t, target_id) }, \
         { "ts", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_node_detected_t, ts) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_node_detected_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_detected_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_detected_t, z) }, \
         { "corx", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_detected_t, corx) }, \
         { "cory", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_node_detected_t, cory) }, \
         { "corz", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_node_detected_t, corz) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_node_detected_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_node_detected_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_node_detected_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_node_detected_t, q3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED { \
    "NODE_DETECTED", \
    12, \
    {  { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 22, offsetof(mavlink_node_detected_t, target_id) }, \
         { "ts", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_node_detected_t, ts) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_node_detected_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_node_detected_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_node_detected_t, z) }, \
         { "corx", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_node_detected_t, corx) }, \
         { "cory", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_node_detected_t, cory) }, \
         { "corz", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_node_detected_t, corz) }, \
         { "q0", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_node_detected_t, q0) }, \
         { "q1", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_node_detected_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_node_detected_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_node_detected_t, q3) }, \
         } \
}
#endif

/**
 * @brief Pack a node_detected message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_id  Target ID of drone
 * @param ts [ms] Timestamp
 * @param x [m] Relative X Position*1000
 * @param y [m] Relative Y Position*1000
 * @param z [m] Relative Z Position*1000
 * @param corx [m] Relative X Position Corvariance*1000
 * @param cory [m] Relative X Position Corvariance*1000
 * @param corz [m] Relative X Position Corvariance*1000
 * @param q0 [m] Relative QUAT W*10000
 * @param q1 [m] Relative QUAT X*10000
 * @param q2 [m] Relative QUAT Y*10000
 * @param q3 [m] Relative QUAT Z*10000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t target_id, int16_t ts, int16_t x, int16_t y, int16_t z, int16_t corx, int16_t cory, int16_t corz, int16_t q0, int16_t q1, int16_t q2, int16_t q3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int16_t(buf, 0, ts);
    _mav_put_int16_t(buf, 2, x);
    _mav_put_int16_t(buf, 4, y);
    _mav_put_int16_t(buf, 6, z);
    _mav_put_int16_t(buf, 8, corx);
    _mav_put_int16_t(buf, 10, cory);
    _mav_put_int16_t(buf, 12, corz);
    _mav_put_int16_t(buf, 14, q0);
    _mav_put_int16_t(buf, 16, q1);
    _mav_put_int16_t(buf, 18, q2);
    _mav_put_int16_t(buf, 20, q3);
    _mav_put_int8_t(buf, 22, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#else
    mavlink_node_detected_t packet;
    packet.ts = ts;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.corx = corx;
    packet.cory = cory;
    packet.corz = corz;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
}

/**
 * @brief Pack a node_detected message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_id  Target ID of drone
 * @param ts [ms] Timestamp
 * @param x [m] Relative X Position*1000
 * @param y [m] Relative Y Position*1000
 * @param z [m] Relative Z Position*1000
 * @param corx [m] Relative X Position Corvariance*1000
 * @param cory [m] Relative X Position Corvariance*1000
 * @param corz [m] Relative X Position Corvariance*1000
 * @param q0 [m] Relative QUAT W*10000
 * @param q1 [m] Relative QUAT X*10000
 * @param q2 [m] Relative QUAT Y*10000
 * @param q3 [m] Relative QUAT Z*10000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t target_id,int16_t ts,int16_t x,int16_t y,int16_t z,int16_t corx,int16_t cory,int16_t corz,int16_t q0,int16_t q1,int16_t q2,int16_t q3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int16_t(buf, 0, ts);
    _mav_put_int16_t(buf, 2, x);
    _mav_put_int16_t(buf, 4, y);
    _mav_put_int16_t(buf, 6, z);
    _mav_put_int16_t(buf, 8, corx);
    _mav_put_int16_t(buf, 10, cory);
    _mav_put_int16_t(buf, 12, corz);
    _mav_put_int16_t(buf, 14, q0);
    _mav_put_int16_t(buf, 16, q1);
    _mav_put_int16_t(buf, 18, q2);
    _mav_put_int16_t(buf, 20, q3);
    _mav_put_int8_t(buf, 22, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#else
    mavlink_node_detected_t packet;
    packet.ts = ts;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.corx = corx;
    packet.cory = cory;
    packet.corz = corz;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
}

/**
 * @brief Encode a node_detected struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_detected_t* node_detected)
{
    return mavlink_msg_node_detected_pack(system_id, component_id, msg, node_detected->target_id, node_detected->ts, node_detected->x, node_detected->y, node_detected->z, node_detected->corx, node_detected->cory, node_detected->corz, node_detected->q0, node_detected->q1, node_detected->q2, node_detected->q3);
}

/**
 * @brief Encode a node_detected struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_detected_t* node_detected)
{
    return mavlink_msg_node_detected_pack_chan(system_id, component_id, chan, msg, node_detected->target_id, node_detected->ts, node_detected->x, node_detected->y, node_detected->z, node_detected->corx, node_detected->cory, node_detected->corz, node_detected->q0, node_detected->q1, node_detected->q2, node_detected->q3);
}

/**
 * @brief Send a node_detected message
 * @param chan MAVLink channel to send the message
 *
 * @param target_id  Target ID of drone
 * @param ts [ms] Timestamp
 * @param x [m] Relative X Position*1000
 * @param y [m] Relative Y Position*1000
 * @param z [m] Relative Z Position*1000
 * @param corx [m] Relative X Position Corvariance*1000
 * @param cory [m] Relative X Position Corvariance*1000
 * @param corz [m] Relative X Position Corvariance*1000
 * @param q0 [m] Relative QUAT W*10000
 * @param q1 [m] Relative QUAT X*10000
 * @param q2 [m] Relative QUAT Y*10000
 * @param q3 [m] Relative QUAT Z*10000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_detected_send(mavlink_channel_t chan, int8_t target_id, int16_t ts, int16_t x, int16_t y, int16_t z, int16_t corx, int16_t cory, int16_t corz, int16_t q0, int16_t q1, int16_t q2, int16_t q3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int16_t(buf, 0, ts);
    _mav_put_int16_t(buf, 2, x);
    _mav_put_int16_t(buf, 4, y);
    _mav_put_int16_t(buf, 6, z);
    _mav_put_int16_t(buf, 8, corx);
    _mav_put_int16_t(buf, 10, cory);
    _mav_put_int16_t(buf, 12, corz);
    _mav_put_int16_t(buf, 14, q0);
    _mav_put_int16_t(buf, 16, q1);
    _mav_put_int16_t(buf, 18, q2);
    _mav_put_int16_t(buf, 20, q3);
    _mav_put_int8_t(buf, 22, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, buf, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#else
    mavlink_node_detected_t packet;
    packet.ts = ts;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.corx = corx;
    packet.cory = cory;
    packet.corz = corz;
    packet.q0 = q0;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)&packet, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}

/**
 * @brief Send a node_detected message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_node_detected_send_struct(mavlink_channel_t chan, const mavlink_node_detected_t* node_detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_node_detected_send(chan, node_detected->target_id, node_detected->ts, node_detected->x, node_detected->y, node_detected->z, node_detected->corx, node_detected->cory, node_detected->corz, node_detected->q0, node_detected->q1, node_detected->q2, node_detected->q3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)node_detected, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}

#if MAVLINK_MSG_ID_NODE_DETECTED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_detected_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t target_id, int16_t ts, int16_t x, int16_t y, int16_t z, int16_t corx, int16_t cory, int16_t corz, int16_t q0, int16_t q1, int16_t q2, int16_t q3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, ts);
    _mav_put_int16_t(buf, 2, x);
    _mav_put_int16_t(buf, 4, y);
    _mav_put_int16_t(buf, 6, z);
    _mav_put_int16_t(buf, 8, corx);
    _mav_put_int16_t(buf, 10, cory);
    _mav_put_int16_t(buf, 12, corz);
    _mav_put_int16_t(buf, 14, q0);
    _mav_put_int16_t(buf, 16, q1);
    _mav_put_int16_t(buf, 18, q2);
    _mav_put_int16_t(buf, 20, q3);
    _mav_put_int8_t(buf, 22, target_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, buf, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#else
    mavlink_node_detected_t *packet = (mavlink_node_detected_t *)msgbuf;
    packet->ts = ts;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->corx = corx;
    packet->cory = cory;
    packet->corz = corz;
    packet->q0 = q0;
    packet->q1 = q1;
    packet->q2 = q2;
    packet->q3 = q3;
    packet->target_id = target_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)packet, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}
#endif

#endif

// MESSAGE NODE_DETECTED UNPACKING


/**
 * @brief Get field target_id from node_detected message
 *
 * @return  Target ID of drone
 */
static inline int8_t mavlink_msg_node_detected_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  22);
}

/**
 * @brief Get field ts from node_detected message
 *
 * @return [ms] Timestamp
 */
static inline int16_t mavlink_msg_node_detected_get_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field x from node_detected message
 *
 * @return [m] Relative X Position*1000
 */
static inline int16_t mavlink_msg_node_detected_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field y from node_detected message
 *
 * @return [m] Relative Y Position*1000
 */
static inline int16_t mavlink_msg_node_detected_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field z from node_detected message
 *
 * @return [m] Relative Z Position*1000
 */
static inline int16_t mavlink_msg_node_detected_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field corx from node_detected message
 *
 * @return [m] Relative X Position Corvariance*1000
 */
static inline int16_t mavlink_msg_node_detected_get_corx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field cory from node_detected message
 *
 * @return [m] Relative X Position Corvariance*1000
 */
static inline int16_t mavlink_msg_node_detected_get_cory(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field corz from node_detected message
 *
 * @return [m] Relative X Position Corvariance*1000
 */
static inline int16_t mavlink_msg_node_detected_get_corz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field q0 from node_detected message
 *
 * @return [m] Relative QUAT W*10000
 */
static inline int16_t mavlink_msg_node_detected_get_q0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field q1 from node_detected message
 *
 * @return [m] Relative QUAT X*10000
 */
static inline int16_t mavlink_msg_node_detected_get_q1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field q2 from node_detected message
 *
 * @return [m] Relative QUAT Y*10000
 */
static inline int16_t mavlink_msg_node_detected_get_q2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field q3 from node_detected message
 *
 * @return [m] Relative QUAT Z*10000
 */
static inline int16_t mavlink_msg_node_detected_get_q3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Decode a node_detected message into a struct
 *
 * @param msg The message to decode
 * @param node_detected C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_detected_decode(const mavlink_message_t* msg, mavlink_node_detected_t* node_detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    node_detected->ts = mavlink_msg_node_detected_get_ts(msg);
    node_detected->x = mavlink_msg_node_detected_get_x(msg);
    node_detected->y = mavlink_msg_node_detected_get_y(msg);
    node_detected->z = mavlink_msg_node_detected_get_z(msg);
    node_detected->corx = mavlink_msg_node_detected_get_corx(msg);
    node_detected->cory = mavlink_msg_node_detected_get_cory(msg);
    node_detected->corz = mavlink_msg_node_detected_get_corz(msg);
    node_detected->q0 = mavlink_msg_node_detected_get_q0(msg);
    node_detected->q1 = mavlink_msg_node_detected_get_q1(msg);
    node_detected->q2 = mavlink_msg_node_detected_get_q2(msg);
    node_detected->q3 = mavlink_msg_node_detected_get_q3(msg);
    node_detected->target_id = mavlink_msg_node_detected_get_target_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NODE_DETECTED_LEN? msg->len : MAVLINK_MSG_ID_NODE_DETECTED_LEN;
        memset(node_detected, 0, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
    memcpy(node_detected, _MAV_PAYLOAD(msg), len);
#endif
}
