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

int get_time_sec()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (tv.tv_sec);
}

int min2sec(float min)
{
    return min*60;
}
typedef struct time
{
    int prev_sec;
    int current_sec;
} time_s;

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

/*void send_position(sts *sts, sockport *sock, int id)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Convert coordinates to MAVLink format
    (void)sts;
    int32_t lat_int = 3777490; //(int32_t)(sts->req_lat[id] * 1e7); // Latitude in degrees * 1E7
    int32_t lon_int = -1224194; //(int32_t)(sts->req_lon[id] * 1e7); // Longitude in degrees * 1E7
    float alt = 100;//sts->req_alt[id]; // Altitude in meters (relative to home)

    // Pack the SET_POSITION_TARGET_GLOBAL_INT message
    mavlink_msg_set_position_target_global_int_pack(
        GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
        DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
        0, // Time boot ms (can be 0 if not used)
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Coordinate frame
        0x0007, // Type mask: Only position is enabled
        lat_int, lon_int, alt, // Latitude, Longitude, Altitude
        0, 0, 0, // Velocity (ignored)
        0, 0, 0, // Acceleration (ignored)
        0, 0 // Yaw, Yaw rate (ignored)
    );

    // Send the message to the autopilot
    len = mavlink_msg_to_send_buffer(buf, &msg);
    int ret = sendto(sock->sockfd[id], buf, len, 0,
                     (struct sockaddr*)&sock->autopilot_addr[id],
                     sizeof(sock->autopilot_addr[id]));
    if (ret == -1) {
        perror("sendto failed");
    } else {
        printf("Position target sent: Lat=%d, Lon=%d, Alt=%.2f\n",
               lat_int, lon_int, alt);
    }
}*/

void limits_bank(sts *sts, sockport *sock, int id)
{
    (void)sts;
    mavlink_param_set_t param_set;
    mavlink_message_t msg;
    uint8_t buf[BUFFER_LENGTH];
    uint16_t len;
    strncpy(param_set.param_id, "ROLL_LIMIT_DEG", 16);
    param_set.param_value = 30.0f;
    param_set.target_system = DEFAULT_TARGET_SYSTEM;
    param_set.target_component = DEFAULT_TARGET_COMPONENT;
    param_set.param_type = MAV_PARAM_TYPE_REAL32;
    mavlink_msg_param_set_encode(GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &param_set);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    int ret = sendto(sock->sockfd[id], buf, len, 0,
                     (struct sockaddr*)&sock->autopilot_addr[id],
                     sizeof(sock->autopilot_addr[id]));
    if (ret == -1) {
        perror("sendto failed");
    } else {
        printf("Bank limit set to 30 degrees for UAV %d\n", id);    
    }
}

void send_reposition(sockport *sock, sts *sts, joy_s *joy,int id) {
    (void)joy;
    mavlink_message_t msg;
    uint8_t buf[BUFFER_LENGTH];
    uint16_t len;
    int32_t lat, lon;
    float alt;
    float lioter = 0.0f;
    if (sts->status != 3 && sts->status != 4)
    {
        if (id ==0)
        {
        // Convert coordinates to fixed-point
            lat = (int32_t)(-35.3708726 * 1e7);
            lon = (int32_t)(149.1726422 * 1e7);
            sts->t_lat[id] = -35.3708726;
            sts->t_lon[id] = 149.1726422;
            alt = 100;  // Altitude in meters
        }
        else
        {
            // Convert coordinates to fixed-point
            lat = (int32_t)(sts->req_lat[id] * 1e7);
            lon = (int32_t)(sts->req_lon[id] * 1e7);
            alt = 100;  // Altitude in meters
        }
    }
    if (!sts->timeout && sts->t2m_distance[0] < 260)
    {
        sts->current_sec = get_time_sec();
        sts->prev_sec = sts->current_sec;
        sts->timeout = true;
    }
    if (sts->t2m_distance[0] < 250 && (sts->current_sec - sts->prev_sec) < min2sec(2))
    {
        printf("reposition after 2 min");
        printf("target to meter distance: %f\n", sts->t2m_distance[0]);
        sts->status = 4;
        lat = (int32_t)(-35.3708726 * 1e7);
        lon = (int32_t)(149.1726422 * 1e7);
        if (id == 0)
            alt = 120;  // Altitude in meters
        else
            alt = 100;  // Altitude in meters
        if (id == 0)
            lioter = sts->loiter[0];
        else
            lioter = sts->loiter[id];
    }
    else if ((sts->current_sec - sts->prev_sec) > min2sec(2)) {
        // Only transition to status 3 if not already in rejoining state
        if (sts->status == 3) {
            limits_bank(sts, sock, 0);
            if (id == 0) {
                lat = (int32_t)(-35.3669006 * 1e7);
                lon = (int32_t)(149.1709042 * 1e7);
            } else {
                lat = (int32_t)(sts->req_lat[id] * 1e7);
                lon = (int32_t)(sts->req_lon[id] * 1e7);
            }
            alt = 100;
        }
        else
        {
            if (id == 0) {
                lat = (int32_t)(-35.3737245 * 1e7);
                lon = (int32_t)(149.1580725 * 1e7);
                sts->t_lat[0] = -35.3737245;
                sts->t_lon[0] = 149.1580725;
                alt = 100;
            } else {
                lat = (int32_t)(sts->req_lat[id] * 1e7);
                lon = (int32_t)(sts->req_lon[id] * 1e7);
                alt = 100;
            }
            gps2meter(sts);
            if (sts->t2m_distance[0] < 350)
                sts->status = 3;
        }
    }
    // Pack the MAV_CMD_DO_REPOSITION command
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
        lioter,                   // Param3: Reserved
        0.0f,                   // Param4: Yaw heading (NaN for unchanged)
        lat,                // Param5: Latitude
        lon,                // Param6: Longitude
        alt                     // Param7: Altitude
    );
    /*mavlink_msg_command_int_pack(
        GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
        DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
        MAV_CMD_DO_REPOSITION,             // Command ID
        0,                                 // Current
        0,                                 // Autocontinue
        -1.0f,                             // Ground speed
        MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, // Reposition flags
        0.0f,                              // Reserved
        NAN,                               // Yaw heading (NaN for unchanged)
        lat,                               // Latitude
        lon,                               // Longitude
        alt                                // Altitude
    );*/

    // Send the command to the autopilot
    len = mavlink_msg_to_send_buffer(buf, &msg);
    int ret = sendto(sock->sockfd[id], buf, len, 0,
                     (struct sockaddr*)&sock->autopilot_addr[id],
                     sizeof(sock->autopilot_addr[id]));
    if (ret == -1) {
        perror("sendto failed");
    }
    sts->current_sec = get_time_sec();

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
           // printf("Setting Plane GUIDED mode\n");
            //send_override(joy, sock, id);
            break;
        case 2: // Set AUTO mode for fixed-wing
            //if (id != 0)
                send_reposition(sock, sts, joy ,id);
            
            break;
    }
   // printf("Send autopilot exited\n");
    return ;
}