/** @file
 *    @brief MAVLink comm protocol testsuite generated from swarm.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef SWARM_TESTSUITE_H
#define SWARM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_swarm(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_swarm(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_node_realtime_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NODE_REALTIME_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_node_realtime_info_t packet_in = {
            17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0,
            {297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0, 305.0, 306.0}, 245, 56
    };
    mavlink_node_realtime_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.q0 = packet_in.q0;
        packet1.q1 = packet_in.q1;
        packet1.q2 = packet_in.q2;
        packet1.q3 = packet_in.q3;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
    packet1.source_id = packet_in.source_id;
        packet1.odom_vaild = packet_in.odom_vaild;
        
        mav_array_memcpy(packet1.remote_distance, packet_in.remote_distance, sizeof(float)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NODE_REALTIME_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_realtime_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_node_realtime_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_realtime_info_pack(system_id, component_id, &msg, packet1.source_id, packet1.odom_vaild, packet1.x,
                                        packet1.y, packet1.z, packet1.q0, packet1.q1, packet1.q2, packet1.q3,
                                        packet1.vx, packet1.vy, packet1.vz, packet1.remote_distance);
    mavlink_msg_node_realtime_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_realtime_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.source_id,
                                             packet1.odom_vaild, packet1.x, packet1.y, packet1.z, packet1.q0,
                                             packet1.q1, packet1.q2, packet1.q3, packet1.vx, packet1.vy, packet1.vz,
                                             packet1.remote_distance);
    mavlink_msg_node_realtime_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_node_realtime_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_realtime_info_send(MAVLINK_COMM_1, packet1.source_id, packet1.odom_vaild, packet1.x, packet1.y,
                                        packet1.z, packet1.q0, packet1.q1, packet1.q2, packet1.q3, packet1.vx,
                                        packet1.vy, packet1.vz, packet1.remote_distance);
    mavlink_msg_node_realtime_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_node_relative_fused(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NODE_RELATIVE_FUSED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_node_relative_fused_t packet_in = {
        17.0,45.0,73.0,101.0,53,120
    };
    mavlink_node_relative_fused_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rel_x = packet_in.rel_x;
        packet1.rel_y = packet_in.rel_y;
        packet1.rel_z = packet_in.rel_z;
        packet1.rel_yaw_offset = packet_in.rel_yaw_offset;
        packet1.source_id = packet_in.source_id;
        packet1.target_id = packet_in.target_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NODE_RELATIVE_FUSED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_relative_fused_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_node_relative_fused_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_relative_fused_pack(system_id, component_id, &msg, packet1.source_id, packet1.target_id,
                                         packet1.rel_x, packet1.rel_y, packet1.rel_z, packet1.rel_yaw_offset);
    mavlink_msg_node_relative_fused_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_relative_fused_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.source_id,
                                              packet1.target_id, packet1.rel_x, packet1.rel_y, packet1.rel_z,
                                              packet1.rel_yaw_offset);
    mavlink_msg_node_relative_fused_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_node_relative_fused_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_relative_fused_send(MAVLINK_COMM_1, packet1.source_id, packet1.target_id, packet1.rel_x,
                                         packet1.rel_y, packet1.rel_z, packet1.rel_yaw_offset);
    mavlink_msg_node_relative_fused_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_swarm_remote_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_swarm_remote_command_t packet_in = {
            963497464, 963497672, 963497880, 963498088, 963498296, 963498504, 963498712, 963498920, 963499128,
            963499336, 125, 192, 3
    };
    mavlink_swarm_remote_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.param5 = packet_in.param5;
        packet1.param6 = packet_in.param6;
        packet1.param7 = packet_in.param7;
        packet1.param8 = packet_in.param8;
    packet1.param9 = packet_in.param9;
    packet1.param10 = packet_in.param10;
    packet1.source_id = packet_in.source_id;
        packet1.target_id = packet_in.target_id;
        packet1.command_type = packet_in.command_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_remote_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_swarm_remote_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_remote_command_pack(system_id, component_id, &msg, packet1.source_id, packet1.target_id,
                                          packet1.command_type, packet1.param1, packet1.param2, packet1.param3,
                                          packet1.param4, packet1.param5, packet1.param6, packet1.param7,
                                          packet1.param8, packet1.param9, packet1.param10);
    mavlink_msg_swarm_remote_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_remote_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.source_id,
                                               packet1.target_id, packet1.command_type, packet1.param1, packet1.param2,
                                               packet1.param3, packet1.param4, packet1.param5, packet1.param6,
                                               packet1.param7, packet1.param8, packet1.param9, packet1.param10);
    mavlink_msg_swarm_remote_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_swarm_remote_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_remote_command_send(MAVLINK_COMM_1, packet1.source_id, packet1.target_id, packet1.command_type,
                                          packet1.param1, packet1.param2, packet1.param3, packet1.param4,
                                          packet1.param5, packet1.param6, packet1.param7, packet1.param8,
                                          packet1.param9, packet1.param10);
    mavlink_msg_swarm_remote_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_node_detected(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg) {
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NODE_DETECTED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t i;
    mavlink_node_detected_t packet_in = {
            17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 125
    };
    mavlink_node_detected_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.x = packet_in.x;
    packet1.y = packet_in.y;
    packet1.z = packet_in.z;
    packet1.corx = packet_in.corx;
    packet1.cory = packet_in.cory;
    packet1.corz = packet_in.corz;
    packet1.q0 = packet_in.q0;
    packet1.q1 = packet_in.q1;
    packet1.q2 = packet_in.q2;
    packet1.q3 = packet_in.q3;
    packet1.source_id = packet_in.source_id;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
       // cope with extensions
       memset(MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN);
    }
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_detected_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_node_detected_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_detected_pack(system_id, component_id, &msg, packet1.source_id, packet1.x, packet1.y, packet1.z,
                                   packet1.corx, packet1.cory, packet1.corz, packet1.q0, packet1.q1, packet1.q2,
                                   packet1.q3);
    mavlink_msg_node_detected_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_detected_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.source_id, packet1.x,
                                        packet1.y, packet1.z, packet1.corx, packet1.cory, packet1.corz, packet1.q0,
                                        packet1.q1, packet1.q2, packet1.q3);
    mavlink_msg_node_detected_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_to_send_buffer(buffer, &msg);
    for (i = 0; i < mavlink_msg_get_send_buffer_length(&msg); i++) {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_node_detected_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_node_detected_send(MAVLINK_COMM_1, packet1.source_id, packet1.x, packet1.y, packet1.z, packet1.corx,
                                   packet1.cory, packet1.corz, packet1.q0, packet1.q1, packet1.q2, packet1.q3);
    mavlink_msg_node_detected_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_drone_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg) {
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DRONE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t i;
    mavlink_drone_status_t packet_in = {
            5, 72, 139, 206, 17, 84, 151
    };
    mavlink_drone_status_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.source_id = packet_in.source_id;
    packet1.flight_status = packet_in.flight_status;
    packet1.control_auth = packet_in.control_auth;
    packet1.commander_mode = packet_in.commander_mode;
    packet1.rc_valid = packet_in.rc_valid;
    packet1.onboard_cmd_valid = packet_in.onboard_cmd_valid;
    packet1.sdk_valid = packet_in.sdk_valid;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
       // cope with extensions
       memset(MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DRONE_STATUS_MIN_LEN);
    }
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_drone_status_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_status_pack(system_id, component_id, &msg, packet1.source_id, packet1.flight_status,
                                  packet1.control_auth, packet1.commander_mode, packet1.rc_valid,
                                  packet1.onboard_cmd_valid, packet1.sdk_valid);
    mavlink_msg_drone_status_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.source_id,
                                       packet1.flight_status, packet1.control_auth, packet1.commander_mode,
                                       packet1.rc_valid, packet1.onboard_cmd_valid, packet1.sdk_valid);
    mavlink_msg_drone_status_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_to_send_buffer(buffer, &msg);
    for (i = 0; i < mavlink_msg_get_send_buffer_length(&msg); i++) {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_drone_status_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_drone_status_send(MAVLINK_COMM_1, packet1.source_id, packet1.flight_status, packet1.control_auth,
                                  packet1.commander_mode, packet1.rc_valid, packet1.onboard_cmd_valid,
                                  packet1.sdk_valid);
    mavlink_msg_drone_status_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_swarm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_node_realtime_info(system_id, component_id, last_msg);
    mavlink_test_node_relative_fused(system_id, component_id, last_msg);
    mavlink_test_swarm_remote_command(system_id, component_id, last_msg);
    mavlink_test_node_detected(system_id, component_id, last_msg);
    mavlink_test_drone_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SWARM_TESTSUITE_H
