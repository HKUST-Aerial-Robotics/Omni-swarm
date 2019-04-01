#pragma once
// MESSAGE SWARM_REMOTE_COMMAND PACKING

#define MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND 402

MAVPACKED(
typedef struct __mavlink_swarm_remote_command_t {
 uint32_t param1; /*< [m] param1*/
 uint32_t param2; /*< [m] param2*/
 uint32_t param3; /*< [m] param3*/
 uint32_t param4; /*< [m] param4*/
 uint32_t param5; /*< [m] param5*/
 uint32_t param6; /*< [m] param6*/
 uint32_t param7; /*< [m] param7*/
 uint32_t param8; /*< [m] param8*/
    uint32_t param9; /*< [m] param8*/
    uint32_t param10; /*< [m] param8*/
    uint8_t source_id; /*<  Source ID of drone*/
 int8_t target_id; /*<  Target ID of drone*/
 uint8_t command_type; /*< [m] Onboard command type*/
}) mavlink_swarm_remote_command_t;

#define MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN 43
#define MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN 43
#define MAVLINK_MSG_ID_402_LEN 43
#define MAVLINK_MSG_ID_402_MIN_LEN 43

#define MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC 182
#define MAVLINK_MSG_ID_402_CRC 182



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SWARM_REMOTE_COMMAND { \
    402, \
    "SWARM_REMOTE_COMMAND", \
    13, \
    {  { "source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_swarm_remote_command_t, source_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 41, offsetof(mavlink_swarm_remote_command_t, target_id) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_swarm_remote_command_t, command_type) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_swarm_remote_command_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_swarm_remote_command_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_swarm_remote_command_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_swarm_remote_command_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_swarm_remote_command_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_swarm_remote_command_t, param6) }, \
         { "param7", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_swarm_remote_command_t, param7) }, \
         { "param8", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_swarm_remote_command_t, param8) }, \
         { "param9", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_swarm_remote_command_t, param9) }, \
         { "param10", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_swarm_remote_command_t, param10) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SWARM_REMOTE_COMMAND { \
    "SWARM_REMOTE_COMMAND", \
    13, \
    {  { "source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_swarm_remote_command_t, source_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT8_T, 0, 41, offsetof(mavlink_swarm_remote_command_t, target_id) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_swarm_remote_command_t, command_type) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_swarm_remote_command_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_swarm_remote_command_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_swarm_remote_command_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_swarm_remote_command_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_swarm_remote_command_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_swarm_remote_command_t, param6) }, \
         { "param7", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_swarm_remote_command_t, param7) }, \
         { "param8", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_swarm_remote_command_t, param8) }, \
         { "param9", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_swarm_remote_command_t, param9) }, \
         { "param10", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_swarm_remote_command_t, param10) }, \
         } \
}
#endif

/**
 * @brief Pack a swarm_remote_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param command_type [m] Onboard command type
 * @param param1 [m] param1
 * @param param2 [m] param2
 * @param param3 [m] param3
 * @param param4 [m] param4
 * @param param5 [m] param5
 * @param param6 [m] param6
 * @param param7 [m] param7
 * @param param8 [m] param8
 * @param param9 [m] param8
 * @param param10 [m] param8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_remote_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                                             uint8_t source_id, int8_t target_id, uint8_t command_type,
                                                             uint32_t param1, uint32_t param2, uint32_t param3,
                                                             uint32_t param4, uint32_t param5, uint32_t param6,
                                                             uint32_t param7, uint32_t param8, uint32_t param9,
                                                             uint32_t param10)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, param1);
    _mav_put_uint32_t(buf, 4, param2);
    _mav_put_uint32_t(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint32_t(buf, 16, param5);
    _mav_put_uint32_t(buf, 20, param6);
    _mav_put_uint32_t(buf, 24, param7);
    _mav_put_uint32_t(buf, 28, param8);
    _mav_put_uint32_t(buf, 32, param9);
    _mav_put_uint32_t(buf, 36, param10);
    _mav_put_uint8_t(buf, 40, source_id);
    _mav_put_int8_t(buf, 41, target_id);
    _mav_put_uint8_t(buf, 42, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN);
#else
    mavlink_swarm_remote_command_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.param8 = param8;
    packet.param9 = param9;
    packet.param10 = param10;
    packet.source_id = source_id;
    packet.target_id = target_id;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
}

/**
 * @brief Pack a swarm_remote_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param command_type [m] Onboard command type
 * @param param1 [m] param1
 * @param param2 [m] param2
 * @param param3 [m] param3
 * @param param4 [m] param4
 * @param param5 [m] param5
 * @param param6 [m] param6
 * @param param7 [m] param7
 * @param param8 [m] param8
 * @param param9 [m] param8
 * @param param10 [m] param8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_remote_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                                                                  mavlink_message_t* msg,
                                                                  uint8_t source_id, int8_t target_id,
                                                                  uint8_t command_type, uint32_t param1,
                                                                  uint32_t param2, uint32_t param3, uint32_t param4,
                                                                  uint32_t param5, uint32_t param6, uint32_t param7,
                                                                  uint32_t param8, uint32_t param9, uint32_t param10)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, param1);
    _mav_put_uint32_t(buf, 4, param2);
    _mav_put_uint32_t(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint32_t(buf, 16, param5);
    _mav_put_uint32_t(buf, 20, param6);
    _mav_put_uint32_t(buf, 24, param7);
    _mav_put_uint32_t(buf, 28, param8);
    _mav_put_uint32_t(buf, 32, param9);
    _mav_put_uint32_t(buf, 36, param10);
    _mav_put_uint8_t(buf, 40, source_id);
    _mav_put_int8_t(buf, 41, target_id);
    _mav_put_uint8_t(buf, 42, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN);
#else
    mavlink_swarm_remote_command_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.param8 = param8;
    packet.param9 = param9;
    packet.param10 = param10;
    packet.source_id = source_id;
    packet.target_id = target_id;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
}

/**
 * @brief Encode a swarm_remote_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param swarm_remote_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_remote_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_swarm_remote_command_t* swarm_remote_command)
{
    return mavlink_msg_swarm_remote_command_pack(system_id, component_id, msg, swarm_remote_command->source_id,
                                                 swarm_remote_command->target_id, swarm_remote_command->command_type,
                                                 swarm_remote_command->param1, swarm_remote_command->param2,
                                                 swarm_remote_command->param3, swarm_remote_command->param4,
                                                 swarm_remote_command->param5, swarm_remote_command->param6,
                                                 swarm_remote_command->param7, swarm_remote_command->param8,
                                                 swarm_remote_command->param9, swarm_remote_command->param10);
}

/**
 * @brief Encode a swarm_remote_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_remote_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_remote_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_swarm_remote_command_t* swarm_remote_command)
{
    return mavlink_msg_swarm_remote_command_pack_chan(system_id, component_id, chan, msg,
                                                      swarm_remote_command->source_id, swarm_remote_command->target_id,
                                                      swarm_remote_command->command_type, swarm_remote_command->param1,
                                                      swarm_remote_command->param2, swarm_remote_command->param3,
                                                      swarm_remote_command->param4, swarm_remote_command->param5,
                                                      swarm_remote_command->param6, swarm_remote_command->param7,
                                                      swarm_remote_command->param8, swarm_remote_command->param9,
                                                      swarm_remote_command->param10);
}

/**
 * @brief Send a swarm_remote_command message
 * @param chan MAVLink channel to send the message
 *
 * @param source_id  Source ID of drone
 * @param target_id  Target ID of drone
 * @param command_type [m] Onboard command type
 * @param param1 [m] param1
 * @param param2 [m] param2
 * @param param3 [m] param3
 * @param param4 [m] param4
 * @param param5 [m] param5
 * @param param6 [m] param6
 * @param param7 [m] param7
 * @param param8 [m] param8
 * @param param9 [m] param8
 * @param param10 [m] param8
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_swarm_remote_command_send(mavlink_channel_t chan, uint8_t source_id, int8_t target_id, uint8_t command_type, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4, uint32_t param5, uint32_t param6, uint32_t param7, uint32_t param8, uint32_t param9, uint32_t param10)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, param1);
    _mav_put_uint32_t(buf, 4, param2);
    _mav_put_uint32_t(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint32_t(buf, 16, param5);
    _mav_put_uint32_t(buf, 20, param6);
    _mav_put_uint32_t(buf, 24, param7);
    _mav_put_uint32_t(buf, 28, param8);
    _mav_put_uint32_t(buf, 32, param9);
    _mav_put_uint32_t(buf, 36, param10);
    _mav_put_uint8_t(buf, 40, source_id);
    _mav_put_int8_t(buf, 41, target_id);
    _mav_put_uint8_t(buf, 42, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND, buf, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
#else
    mavlink_swarm_remote_command_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.param8 = param8;
    packet.param9 = param9;
    packet.param10 = param10;
    packet.source_id = source_id;
    packet.target_id = target_id;
    packet.command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
#endif
}

/**
 * @brief Send a swarm_remote_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_swarm_remote_command_send_struct(mavlink_channel_t chan, const mavlink_swarm_remote_command_t* swarm_remote_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_swarm_remote_command_send(chan, swarm_remote_command->source_id, swarm_remote_command->target_id, swarm_remote_command->command_type, swarm_remote_command->param1, swarm_remote_command->param2, swarm_remote_command->param3, swarm_remote_command->param4, swarm_remote_command->param5, swarm_remote_command->param6, swarm_remote_command->param7, swarm_remote_command->param8, swarm_remote_command->param9, swarm_remote_command->param10);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND, (const char *)swarm_remote_command, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_swarm_remote_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t source_id, int8_t target_id, uint8_t command_type, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4, uint32_t param5, uint32_t param6, uint32_t param7, uint32_t param8, uint32_t param9, uint32_t param10)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, param1);
    _mav_put_uint32_t(buf, 4, param2);
    _mav_put_uint32_t(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint32_t(buf, 16, param5);
    _mav_put_uint32_t(buf, 20, param6);
    _mav_put_uint32_t(buf, 24, param7);
    _mav_put_uint32_t(buf, 28, param8);
    _mav_put_uint32_t(buf, 32, param9);
    _mav_put_uint32_t(buf, 36, param10);
    _mav_put_uint8_t(buf, 40, source_id);
    _mav_put_int8_t(buf, 41, target_id);
    _mav_put_uint8_t(buf, 42, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND, buf, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
#else
    mavlink_swarm_remote_command_t *packet = (mavlink_swarm_remote_command_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;
    packet->param8 = param8;
    packet->param9 = param9;
    packet->param10 = param10;
    packet->source_id = source_id;
    packet->target_id = target_id;
    packet->command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND, (const char *)packet, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE SWARM_REMOTE_COMMAND UNPACKING


/**
 * @brief Get field source_id from swarm_remote_command message
 *
 * @return  Source ID of drone
 */
static inline uint8_t mavlink_msg_swarm_remote_command_get_source_id(const mavlink_message_t *msg) {
    return _MAV_RETURN_uint8_t(msg, 40);
}

/**
 * @brief Get field target_id from swarm_remote_command message
 *
 * @return  Target ID of drone
 */
static inline int8_t mavlink_msg_swarm_remote_command_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg, 41);
}

/**
 * @brief Get field command_type from swarm_remote_command message
 *
 * @return [m] Onboard command type
 */
static inline uint8_t mavlink_msg_swarm_remote_command_get_command_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg, 42);
}

/**
 * @brief Get field param1 from swarm_remote_command message
 *
 * @return [m] param1
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param2 from swarm_remote_command message
 *
 * @return [m] param2
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field param3 from swarm_remote_command message
 *
 * @return [m] param3
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field param4 from swarm_remote_command message
 *
 * @return [m] param4
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field param5 from swarm_remote_command message
 *
 * @return [m] param5
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field param6 from swarm_remote_command message
 *
 * @return [m] param6
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field param7 from swarm_remote_command message
 *
 * @return [m] param7
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field param8 from swarm_remote_command message
 *
 * @return [m] param8
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field param9 from swarm_remote_command message
 *
 * @return [m] param8
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param9(const mavlink_message_t *msg) {
    return _MAV_RETURN_uint32_t(msg, 32);
}

/**
 * @brief Get field param10 from swarm_remote_command message
 *
 * @return [m] param8
 */
static inline uint32_t mavlink_msg_swarm_remote_command_get_param10(const mavlink_message_t *msg) {
    return _MAV_RETURN_uint32_t(msg, 36);
}

/**
 * @brief Decode a swarm_remote_command message into a struct
 *
 * @param msg The message to decode
 * @param swarm_remote_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_swarm_remote_command_decode(const mavlink_message_t* msg, mavlink_swarm_remote_command_t* swarm_remote_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    swarm_remote_command->param1 = mavlink_msg_swarm_remote_command_get_param1(msg);
    swarm_remote_command->param2 = mavlink_msg_swarm_remote_command_get_param2(msg);
    swarm_remote_command->param3 = mavlink_msg_swarm_remote_command_get_param3(msg);
    swarm_remote_command->param4 = mavlink_msg_swarm_remote_command_get_param4(msg);
    swarm_remote_command->param5 = mavlink_msg_swarm_remote_command_get_param5(msg);
    swarm_remote_command->param6 = mavlink_msg_swarm_remote_command_get_param6(msg);
    swarm_remote_command->param7 = mavlink_msg_swarm_remote_command_get_param7(msg);
    swarm_remote_command->param8 = mavlink_msg_swarm_remote_command_get_param8(msg);
    swarm_remote_command->param9 = mavlink_msg_swarm_remote_command_get_param9(msg);
    swarm_remote_command->param10 = mavlink_msg_swarm_remote_command_get_param10(msg);
    swarm_remote_command->source_id = mavlink_msg_swarm_remote_command_get_source_id(msg);
    swarm_remote_command->target_id = mavlink_msg_swarm_remote_command_get_target_id(msg);
    swarm_remote_command->command_type = mavlink_msg_swarm_remote_command_get_command_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN;
        memset(swarm_remote_command, 0, MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_LEN);
    memcpy(swarm_remote_command, _MAV_PAYLOAD(msg), len);
#endif
}
