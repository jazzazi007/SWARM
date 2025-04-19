/*
 * File: send_autopilot.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file recieves the data from the GNC to send it to the autopilot.
 *              it depends on the GNC calculation after getting the required from the UAS.
 *
 */

#include "../include/swarm.h"

#include <time.h>
//#define _POSIX_C_SOURCE 199309L

// If CLOCK_MONOTONIC is not defined, define it
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
// Add these mode definitions at the top
#define PLANE_MODE_MANUAL    0
#define PLANE_MODE_GUIDED    15
#define PLANE_MODE_AUTO      10
#define PLANE_MODE_RTL       11
#define PLANE_MODE_FBWA      5


void reposition(sockport *sock, sts *sts, int id)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Convert coordinates to fixed-point
    int32_t lat_int = (int32_t)(sts->req_lat[id] * 1e7);
    int32_t lon_int = (int32_t)(sts->req_lon[id] * 1e7);
    float alt = sts->heading_alt_uav[id];  // Altitude in meters
    // Pack COMMAND_INT message
    mavlink_msg_command_int_pack(
        GCS_SYSTEM_ID,           // Source system
        GCS_COMPONENT_ID,        // Source component
        &msg,
        DEFAULT_TARGET_SYSTEM,   // Target system
        DEFAULT_TARGET_COMPONENT,// Target component
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
        MAV_CMD_DO_REPOSITION,   // Command ID
        0,                       // Current
        0,                       // Autocontinue
        -1.0f,                  // Param1: Ground speed
        MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, // Param2: Reposition flags
        0.0f,                   // Param3: Reserved
        0.0f,                   // Param4: Yaw heading (NaN for unchanged)
        lat_int,                // Param5: Latitude
        lon_int,                // Param6: Longitude
        alt                     // Param7: Altitude
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
   // printf("Sending COMMAND_INT - lat: %d, lon: %d, alt: %.2f\n", 
      //     lat_int, lon_int, alt);
    int ret = sendto(sock->sockfd[id], buf, len, 0, 
                    (struct sockaddr*)&sock->autopilot_addr[id], 
                    sizeof(sock->autopilot_addr[id]));
    if (ret == -1) {
        perror("sendto failed");
    } 
}

void send_override(joy_s *joy, sockport *sock, int id)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    mavlink_rc_channels_override_t rc_override;
    uint16_t len;
    //joy_s joy;
    //sockport sock;
    //printf("rc: %d, %d, %d, %d\n", joy->roll, joy->pitch, joy->throttle, joy->yaw);

    rc_override.chan1_raw = joy->roll[id];    // Roll
    rc_override.chan2_raw = joy->pitch[id];    // Pitch
    rc_override.chan3_raw = joy->throttle[id];    // Throttle
    rc_override.chan4_raw = joy->yaw[id];    // Yaw
    rc_override.chan5_raw = 0;
    rc_override.chan6_raw = 0;
    rc_override.chan7_raw = 0;
    rc_override.chan8_raw = 0;
    rc_override.target_system = DEFAULT_TARGET_SYSTEM;
    rc_override.target_component = DEFAULT_TARGET_COMPONENT;

    mavlink_msg_rc_channels_override_encode(GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &rc_override);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock->sockfd[id], buf, len, 0, 
                    (struct sockaddr*)&sock->autopilot_addr[id], 
                    sizeof(sock->autopilot_addr[id]));
    //printf("Sent RC_OVERRIDE message\n");
}

void send_position(sts *sts, sockport *sock, int id)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Pack the SET_POSITION_TARGET_GLOBAL_INT message
    mavlink_msg_set_position_target_global_int_pack(
        GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
        DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
        0, // Time boot ms
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Coordinate frame
        0x0007, // Type mask (only positions enabled)
        sts->req_lat[id] * 1e7, // Latitude
        sts->req_lon[id] * 1e7, // Longitude
        sts->req_alt[id], // Altitude
        0, 0, 0, // Velocity (not used)
        0, 0, 0, // Acceleration (not used)
        0, 0); // Yaw and yaw rate (not used)

    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock->sockfd[id], buf, len, 0,
            (struct sockaddr*)&sock->autopilot_addr[id], 
            sizeof(sock->autopilot_addr[id]));
}

void send_autopilot(sockport *sock, sts *sts, joy_s *joy, int id)
{
    (void)joy;
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    switch (sts->mission_state) {
        case 0: // Do nothing
        mavlink_msg_command_long_pack(
            GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
            DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
            MAV_CMD_DO_SET_MODE, 0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // Base mode
            PLANE_MODE_RTL,                 // Custom mode for plane
            0, 0, 0, 0, 0                     // Additional params
        );
        len = mavlink_msg_to_send_buffer(buf, &msg);
        int ret = sendto(sock->sockfd[id], buf, len, 0, 
                (struct sockaddr*)&sock->autopilot_addr[id], 
                sizeof(sock->autopilot_addr[id]));
        if (ret == -1) {
            perror("sendto failed");
        }
       // printf("Setting Plane RTL mode\n");
            //usleep(1000000);
            break;
        case 1: // Set GUIDED mode for fixed-wing
            mavlink_msg_command_long_pack(
                GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
                DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
                MAV_CMD_DO_SET_MODE, 0,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // Base mode
                PLANE_MODE_GUIDED,                 // Custom mode for plane
                0, 0, 0, 0, 0                     // Additional params
            );
            len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock->sockfd[id], buf, len, 0, 
                    (struct sockaddr*)&sock->autopilot_addr[id], 
                    sizeof(sock->autopilot_addr[id]));
            printf("Setting Plane GUIDED mode\n");
            //send_override(joy, sock, id);
            send_position(sts, sock, id);
            break;
    }
   // printf("Send autopilot exited\n");
    return ;
}