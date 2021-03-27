#pragma once
// MESSAGE DRONE_STATUS PACKING

#define MAVLINK_MSG_ID_DRONE_STATUS 204


typedef struct __mavlink_drone_status_t {
 int32_t lps_time; /*< [ms] LPS_TIME*/
 float vo_latency; /*<  VO LATENCT*/
 float bat_vol; /*<  BATTERY_VOL*/
 float bat_remain; /*< [s] BATTERY_REMAIN*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float yaw; /*< [deg] Yaw*/
 uint8_t flight_status; /*<  Flight status of drone*/
 uint8_t control_auth; /*<  Control AUTH of drone*/
 uint8_t commander_mode; /*<  COMMANDER MODE*/
 uint8_t input_mode; /*<  CTRL INPUT MODE*/
 uint8_t rc_valid; /*<  RC VAILD*/
 uint8_t onboard_cmd_valid; /*<  ONBOARD CMD VALID*/
 uint8_t sdk_valid; /*<  SDK VALID*/
 uint8_t vo_valid; /*<   VO VALID*/
} mavlink_drone_status_t;

#define MAVLINK_MSG_ID_DRONE_STATUS_LEN 40
#define MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN 40
#define MAVLINK_MSG_ID_204_LEN 40
#define MAVLINK_MSG_ID_204_MIN_LEN 40

#define MAVLINK_MSG_ID_DRONE_STATUS_CRC 80
#define MAVLINK_MSG_ID_204_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_STATUS { \
    204, \
    "DRONE_STATUS", \
    16, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_status_t, lps_time) }, \
         { "flight_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_drone_status_t, flight_status) }, \
         { "control_auth", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_drone_status_t, control_auth) }, \
         { "commander_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_drone_status_t, commander_mode) }, \
         { "input_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_drone_status_t, input_mode) }, \
         { "rc_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_drone_status_t, rc_valid) }, \
         { "onboard_cmd_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_drone_status_t, onboard_cmd_valid) }, \
         { "sdk_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_drone_status_t, sdk_valid) }, \
         { "vo_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_drone_status_t, vo_valid) }, \
         { "vo_latency", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_drone_status_t, vo_latency) }, \
         { "bat_vol", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_drone_status_t, bat_vol) }, \
         { "bat_remain", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_drone_status_t, bat_remain) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_status_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_drone_status_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_STATUS { \
    "DRONE_STATUS", \
    16, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_drone_status_t, lps_time) }, \
         { "flight_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_drone_status_t, flight_status) }, \
         { "control_auth", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_drone_status_t, control_auth) }, \
         { "commander_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_drone_status_t, commander_mode) }, \
         { "input_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_drone_status_t, input_mode) }, \
         { "rc_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_drone_status_t, rc_valid) }, \
         { "onboard_cmd_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_drone_status_t, onboard_cmd_valid) }, \
         { "sdk_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_drone_status_t, sdk_valid) }, \
         { "vo_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_drone_status_t, vo_valid) }, \
         { "vo_latency", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_drone_status_t, vo_latency) }, \
         { "bat_vol", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_drone_status_t, bat_vol) }, \
         { "bat_remain", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_drone_status_t, bat_remain) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_drone_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_drone_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_drone_status_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_drone_status_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param flight_status  Flight status of drone
 * @param control_auth  Control AUTH of drone
 * @param commander_mode  COMMANDER MODE
 * @param input_mode  CTRL INPUT MODE
 * @param rc_valid  RC VAILD
 * @param onboard_cmd_valid  ONBOARD CMD VALID
 * @param sdk_valid  SDK VALID
 * @param vo_valid   VO VALID
 * @param vo_latency  VO LATENCT
 * @param bat_vol  BATTERY_VOL
 * @param bat_remain [s] BATTERY_REMAIN
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param yaw [deg] Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, uint8_t flight_status, uint8_t control_auth, uint8_t commander_mode, uint8_t input_mode, uint8_t rc_valid, uint8_t onboard_cmd_valid, uint8_t sdk_valid, uint8_t vo_valid, float vo_latency, float bat_vol, float bat_remain, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_float(buf, 4, vo_latency);
    _mav_put_float(buf, 8, bat_vol);
    _mav_put_float(buf, 12, bat_remain);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, flight_status);
    _mav_put_uint8_t(buf, 33, control_auth);
    _mav_put_uint8_t(buf, 34, commander_mode);
    _mav_put_uint8_t(buf, 35, input_mode);
    _mav_put_uint8_t(buf, 36, rc_valid);
    _mav_put_uint8_t(buf, 37, onboard_cmd_valid);
    _mav_put_uint8_t(buf, 38, sdk_valid);
    _mav_put_uint8_t(buf, 39, vo_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_STATUS_LEN);
#else
    mavlink_drone_status_t packet;
    packet.lps_time = lps_time;
    packet.vo_latency = vo_latency;
    packet.bat_vol = bat_vol;
    packet.bat_remain = bat_remain;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flight_status = flight_status;
    packet.control_auth = control_auth;
    packet.commander_mode = commander_mode;
    packet.input_mode = input_mode;
    packet.rc_valid = rc_valid;
    packet.onboard_cmd_valid = onboard_cmd_valid;
    packet.sdk_valid = sdk_valid;
    packet.vo_valid = vo_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
}

/**
 * @brief Pack a drone_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param flight_status  Flight status of drone
 * @param control_auth  Control AUTH of drone
 * @param commander_mode  COMMANDER MODE
 * @param input_mode  CTRL INPUT MODE
 * @param rc_valid  RC VAILD
 * @param onboard_cmd_valid  ONBOARD CMD VALID
 * @param sdk_valid  SDK VALID
 * @param vo_valid   VO VALID
 * @param vo_latency  VO LATENCT
 * @param bat_vol  BATTERY_VOL
 * @param bat_remain [s] BATTERY_REMAIN
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param yaw [deg] Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,uint8_t flight_status,uint8_t control_auth,uint8_t commander_mode,uint8_t input_mode,uint8_t rc_valid,uint8_t onboard_cmd_valid,uint8_t sdk_valid,uint8_t vo_valid,float vo_latency,float bat_vol,float bat_remain,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_float(buf, 4, vo_latency);
    _mav_put_float(buf, 8, bat_vol);
    _mav_put_float(buf, 12, bat_remain);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, flight_status);
    _mav_put_uint8_t(buf, 33, control_auth);
    _mav_put_uint8_t(buf, 34, commander_mode);
    _mav_put_uint8_t(buf, 35, input_mode);
    _mav_put_uint8_t(buf, 36, rc_valid);
    _mav_put_uint8_t(buf, 37, onboard_cmd_valid);
    _mav_put_uint8_t(buf, 38, sdk_valid);
    _mav_put_uint8_t(buf, 39, vo_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_STATUS_LEN);
#else
    mavlink_drone_status_t packet;
    packet.lps_time = lps_time;
    packet.vo_latency = vo_latency;
    packet.bat_vol = bat_vol;
    packet.bat_remain = bat_remain;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flight_status = flight_status;
    packet.control_auth = control_auth;
    packet.commander_mode = commander_mode;
    packet.input_mode = input_mode;
    packet.rc_valid = rc_valid;
    packet.onboard_cmd_valid = onboard_cmd_valid;
    packet.sdk_valid = sdk_valid;
    packet.vo_valid = vo_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
}

/**
 * @brief Encode a drone_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_status_t* drone_status)
{
    return mavlink_msg_drone_status_pack(system_id, component_id, msg, drone_status->lps_time, drone_status->flight_status, drone_status->control_auth, drone_status->commander_mode, drone_status->input_mode, drone_status->rc_valid, drone_status->onboard_cmd_valid, drone_status->sdk_valid, drone_status->vo_valid, drone_status->vo_latency, drone_status->bat_vol, drone_status->bat_remain, drone_status->x, drone_status->y, drone_status->z, drone_status->yaw);
}

/**
 * @brief Encode a drone_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_status_t* drone_status)
{
    return mavlink_msg_drone_status_pack_chan(system_id, component_id, chan, msg, drone_status->lps_time, drone_status->flight_status, drone_status->control_auth, drone_status->commander_mode, drone_status->input_mode, drone_status->rc_valid, drone_status->onboard_cmd_valid, drone_status->sdk_valid, drone_status->vo_valid, drone_status->vo_latency, drone_status->bat_vol, drone_status->bat_remain, drone_status->x, drone_status->y, drone_status->z, drone_status->yaw);
}

/**
 * @brief Send a drone_status message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param flight_status  Flight status of drone
 * @param control_auth  Control AUTH of drone
 * @param commander_mode  COMMANDER MODE
 * @param input_mode  CTRL INPUT MODE
 * @param rc_valid  RC VAILD
 * @param onboard_cmd_valid  ONBOARD CMD VALID
 * @param sdk_valid  SDK VALID
 * @param vo_valid   VO VALID
 * @param vo_latency  VO LATENCT
 * @param bat_vol  BATTERY_VOL
 * @param bat_remain [s] BATTERY_REMAIN
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param yaw [deg] Yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_status_send(mavlink_channel_t chan, int32_t lps_time, uint8_t flight_status, uint8_t control_auth, uint8_t commander_mode, uint8_t input_mode, uint8_t rc_valid, uint8_t onboard_cmd_valid, uint8_t sdk_valid, uint8_t vo_valid, float vo_latency, float bat_vol, float bat_remain, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_float(buf, 4, vo_latency);
    _mav_put_float(buf, 8, bat_vol);
    _mav_put_float(buf, 12, bat_remain);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, flight_status);
    _mav_put_uint8_t(buf, 33, control_auth);
    _mav_put_uint8_t(buf, 34, commander_mode);
    _mav_put_uint8_t(buf, 35, input_mode);
    _mav_put_uint8_t(buf, 36, rc_valid);
    _mav_put_uint8_t(buf, 37, onboard_cmd_valid);
    _mav_put_uint8_t(buf, 38, sdk_valid);
    _mav_put_uint8_t(buf, 39, vo_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_STATUS, buf, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
#else
    mavlink_drone_status_t packet;
    packet.lps_time = lps_time;
    packet.vo_latency = vo_latency;
    packet.bat_vol = bat_vol;
    packet.bat_remain = bat_remain;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.flight_status = flight_status;
    packet.control_auth = control_auth;
    packet.commander_mode = commander_mode;
    packet.input_mode = input_mode;
    packet.rc_valid = rc_valid;
    packet.onboard_cmd_valid = onboard_cmd_valid;
    packet.sdk_valid = sdk_valid;
    packet.vo_valid = vo_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
#endif
}

/**
 * @brief Send a drone_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_status_send_struct(mavlink_channel_t chan, const mavlink_drone_status_t* drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_status_send(chan, drone_status->lps_time, drone_status->flight_status, drone_status->control_auth, drone_status->commander_mode, drone_status->input_mode, drone_status->rc_valid, drone_status->onboard_cmd_valid, drone_status->sdk_valid, drone_status->vo_valid, drone_status->vo_latency, drone_status->bat_vol, drone_status->bat_remain, drone_status->x, drone_status->y, drone_status->z, drone_status->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_STATUS, (const char *)drone_status, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, uint8_t flight_status, uint8_t control_auth, uint8_t commander_mode, uint8_t input_mode, uint8_t rc_valid, uint8_t onboard_cmd_valid, uint8_t sdk_valid, uint8_t vo_valid, float vo_latency, float bat_vol, float bat_remain, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lps_time);
    _mav_put_float(buf, 4, vo_latency);
    _mav_put_float(buf, 8, bat_vol);
    _mav_put_float(buf, 12, bat_remain);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, flight_status);
    _mav_put_uint8_t(buf, 33, control_auth);
    _mav_put_uint8_t(buf, 34, commander_mode);
    _mav_put_uint8_t(buf, 35, input_mode);
    _mav_put_uint8_t(buf, 36, rc_valid);
    _mav_put_uint8_t(buf, 37, onboard_cmd_valid);
    _mav_put_uint8_t(buf, 38, sdk_valid);
    _mav_put_uint8_t(buf, 39, vo_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_STATUS, buf, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
#else
    mavlink_drone_status_t *packet = (mavlink_drone_status_t *)msgbuf;
    packet->lps_time = lps_time;
    packet->vo_latency = vo_latency;
    packet->bat_vol = bat_vol;
    packet->bat_remain = bat_remain;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->flight_status = flight_status;
    packet->control_auth = control_auth;
    packet->commander_mode = commander_mode;
    packet->input_mode = input_mode;
    packet->rc_valid = rc_valid;
    packet->onboard_cmd_valid = onboard_cmd_valid;
    packet->sdk_valid = sdk_valid;
    packet->vo_valid = vo_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_STATUS, (const char *)packet, MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRONE_STATUS_LEN, MAVLINK_MSG_ID_DRONE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_STATUS UNPACKING


/**
 * @brief Get field lps_time from drone_status message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_drone_status_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field flight_status from drone_status message
 *
 * @return  Flight status of drone
 */
static inline uint8_t mavlink_msg_drone_status_get_flight_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field control_auth from drone_status message
 *
 * @return  Control AUTH of drone
 */
static inline uint8_t mavlink_msg_drone_status_get_control_auth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field commander_mode from drone_status message
 *
 * @return  COMMANDER MODE
 */
static inline uint8_t mavlink_msg_drone_status_get_commander_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field input_mode from drone_status message
 *
 * @return  CTRL INPUT MODE
 */
static inline uint8_t mavlink_msg_drone_status_get_input_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field rc_valid from drone_status message
 *
 * @return  RC VAILD
 */
static inline uint8_t mavlink_msg_drone_status_get_rc_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field onboard_cmd_valid from drone_status message
 *
 * @return  ONBOARD CMD VALID
 */
static inline uint8_t mavlink_msg_drone_status_get_onboard_cmd_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field sdk_valid from drone_status message
 *
 * @return  SDK VALID
 */
static inline uint8_t mavlink_msg_drone_status_get_sdk_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field vo_valid from drone_status message
 *
 * @return   VO VALID
 */
static inline uint8_t mavlink_msg_drone_status_get_vo_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field vo_latency from drone_status message
 *
 * @return  VO LATENCT
 */
static inline float mavlink_msg_drone_status_get_vo_latency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field bat_vol from drone_status message
 *
 * @return  BATTERY_VOL
 */
static inline float mavlink_msg_drone_status_get_bat_vol(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field bat_remain from drone_status message
 *
 * @return [s] BATTERY_REMAIN
 */
static inline float mavlink_msg_drone_status_get_bat_remain(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from drone_status message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_drone_status_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from drone_status message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_drone_status_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from drone_status message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_drone_status_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from drone_status message
 *
 * @return [deg] Yaw
 */
static inline float mavlink_msg_drone_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a drone_status message into a struct
 *
 * @param msg The message to decode
 * @param drone_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_status_decode(const mavlink_message_t* msg, mavlink_drone_status_t* drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_status->lps_time = mavlink_msg_drone_status_get_lps_time(msg);
    drone_status->vo_latency = mavlink_msg_drone_status_get_vo_latency(msg);
    drone_status->bat_vol = mavlink_msg_drone_status_get_bat_vol(msg);
    drone_status->bat_remain = mavlink_msg_drone_status_get_bat_remain(msg);
    drone_status->x = mavlink_msg_drone_status_get_x(msg);
    drone_status->y = mavlink_msg_drone_status_get_y(msg);
    drone_status->z = mavlink_msg_drone_status_get_z(msg);
    drone_status->yaw = mavlink_msg_drone_status_get_yaw(msg);
    drone_status->flight_status = mavlink_msg_drone_status_get_flight_status(msg);
    drone_status->control_auth = mavlink_msg_drone_status_get_control_auth(msg);
    drone_status->commander_mode = mavlink_msg_drone_status_get_commander_mode(msg);
    drone_status->input_mode = mavlink_msg_drone_status_get_input_mode(msg);
    drone_status->rc_valid = mavlink_msg_drone_status_get_rc_valid(msg);
    drone_status->onboard_cmd_valid = mavlink_msg_drone_status_get_onboard_cmd_valid(msg);
    drone_status->sdk_valid = mavlink_msg_drone_status_get_sdk_valid(msg);
    drone_status->vo_valid = mavlink_msg_drone_status_get_vo_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_DRONE_STATUS_LEN;
        memset(drone_status, 0, MAVLINK_MSG_ID_DRONE_STATUS_LEN);
    memcpy(drone_status, _MAV_PAYLOAD(msg), len);
#endif
}
