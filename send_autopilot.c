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

#include "../include/gnc.h"

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


typedef struct {
    double lat;
    double lon;
    float alt;
} waypoint_t;


bool wait_for_ack(sockport *sock, uint16_t command_id, int timeout_us) {
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    mavlink_status_t status;
    fd_set readfds;
    struct timeval tv;
    int maxfd = sock->sockfd + 1;

    // Set timeout
    tv.tv_sec = 0;
    tv.tv_usec = timeout_us;

    while (timeout_us > 0) {
        FD_ZERO(&readfds);
        FD_SET(sock->sockfd, &readfds);

        int ret = select(maxfd, &readfds, NULL, NULL, &tv);
        if (ret > 0) {
            ssize_t recvlen = recvfrom(sock->sockfd, buf, BUFFER_LENGTH, 0, NULL, NULL);
            if (recvlen > 0) {
                for (int i = 0; i < recvlen; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        // Check for COMMAND_ACK
                        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            if (ack.command == command_id) {
                                if (ack.result == MAV_RESULT_ACCEPTED) {
                                    printf("Command %d acknowledged\n", command_id);
                                    return true;
                                } else {
                                    printf("Command %d failed with result %d\n", 
                                           command_id, ack.result);
                                    return false;
                                }
                            }
                        }
                        // Check for MISSION_ACK
                        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
                            mavlink_mission_ack_t ack;
                            mavlink_msg_mission_ack_decode(&msg, &ack);
                            if (ack.type == MAV_MISSION_ACCEPTED) {
                                printf("Mission accepted\n");
                                return true;
                            } else {
                                printf("Mission failed with result %d\n", ack.type);
                                return false;
                            }
                        }
                    }
                }
            }
        }
        //usleep(10000);  // 10ms sleep
        timeout_us -= 10000;
    }
    printf("Timeout waiting for acknowledgment\n");
    return false;
}

int send_mavlink_message(int sockfd, struct sockaddr_in* target_addr) {
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Pack heartbeat message
    mavlink_msg_heartbeat_pack(
        GCS_SYSTEM_ID,           // Source system
        GCS_COMPONENT_ID,        // Source component
        &msg,
        MAV_TYPE_GCS,           // Type = Ground Control Station
        MAV_AUTOPILOT_INVALID,  // Autopilot type
        MAV_MODE_MANUAL_ARMED,  // System mode
        0,                      // Custom mode
        MAV_STATE_ACTIVE       // System state
    );

    // Copy message to send buffer
    len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send message
    ssize_t bytes_sent = sendto(sockfd, buf, len, 0, 
                               (struct sockaddr*)target_addr, 
                               sizeof(struct sockaddr_in));
    
    if (bytes_sent == -1) {
        perror("Error sending message");
        return -1;
    }

    printf("Sent message with length: %d\n", len);
    return 0;
}

// Modify send_mission to use acknowledgments
int send_mission(sockport *sock, sts *sts) {
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;
    const int num_waypoints = 4;
    mavlink_mission_item_int_t mission_item;

    for (int retry = 0; retry < MISSION_RETRY_COUNT; retry++) 
    {

        mavlink_msg_mission_clear_all_pack(
            GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
            DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
            MAV_MISSION_TYPE_MISSION
        );
        
        // Send mission count
        mavlink_msg_mission_count_pack(
            GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
            DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
            5, MAV_MISSION_TYPE_MISSION, 0
        );

        len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(sock->sockfd, buf, len, 0, 
               (struct sockaddr*)&sock->autopilot_addr, 
               sizeof(sock->autopilot_addr));
        
       // printf("Sending mission count: %d waypoints (Retry %d)\n", num_waypoints, retry + 1);

        // Wait for first mission request (seq=0)
        memset(&mission_item, 0, sizeof(mission_item));
        mission_item.target_system = DEFAULT_TARGET_SYSTEM;
        mission_item.target_component = DEFAULT_TARGET_COMPONENT;
        mission_item.seq = 0;  // First waypoint
        mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        mission_item.command = MAV_CMD_NAV_WAYPOINT;
        mission_item.current = 1;  // This is the current waypoint
        mission_item.autocontinue = 1;
        mission_item.param1 = 0;        // No hold time
        mission_item.param2 = 100.0;    // Acceptance radius for plane
        mission_item.param3 = 0;        // Pass through waypoint
        mission_item.param4 = 0;        // No specific yaw
        mission_item.x = (int32_t)(sts->stable_lat[0] * 1e7);    
        mission_item.y = (int32_t)(sts->stable_lon[0] * 1e7);    
        mission_item.z = sts->req_alt;  
        mission_item.mission_type = MAV_MISSION_TYPE_MISSION;

        mavlink_msg_mission_item_int_encode(
            GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &mission_item
        );

        len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(sock->sockfd, buf, len, 0, 
               (struct sockaddr*)&sock->autopilot_addr, 
               sizeof(sock->autopilot_addr));

        //printf("Sent waypoint 0: lat=%.7f, lon=%.7f, alt=%.2f\n",
            //   sts->home_lat, sts->home_lon, sts->req_alt);
        for (int i = 0; i < num_waypoints; i++) 
        {
            // Send first waypoint
            memset(&mission_item, 0, sizeof(mission_item));
            mission_item.target_system = DEFAULT_TARGET_SYSTEM;
            mission_item.target_component = DEFAULT_TARGET_COMPONENT;
            mission_item.seq = i + 1;  // First waypoint
            mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            mission_item.command = MAV_CMD_NAV_WAYPOINT;
            mission_item.current = 0;  // This is the current waypoint
            mission_item.autocontinue = 1;
            mission_item.param1 = 0;        // No hold time
            mission_item.param2 = 100.0;    // Acceptance radius for plane
            mission_item.param3 = 0;        // Pass through waypoint
            mission_item.param4 = 0;        // No specific yaw
            mission_item.x = (int32_t)(sts->stable_lat[i] * 1e7);    
            mission_item.y = (int32_t)(sts->stable_lon[i] * 1e7);    
            mission_item.z = sts->stable_alt[i];  
            mission_item.mission_type = MAV_MISSION_TYPE_MISSION;

            mavlink_msg_mission_item_int_encode(
                GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &mission_item
            );

            len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock->sockfd, buf, len, 0, 
                   (struct sockaddr*)&sock->autopilot_addr, 
                   sizeof(sock->autopilot_addr));

           // printf("Sent waypoint 0: lat=%.7f, lon=%.7f, alt=%.2f\n",
                  // sts->req_lat, sts->req_lon, sts->req_alt);
        //printf("Mission upload failed, retrying...\n");
        }
    }

    return -1;
}

void reposition(sockport *sock, sts *sts)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;

    // Convert coordinates to fixed-point
    int32_t lat_int = (int32_t)(sts->req_lat * 1e7);
    int32_t lon_int = (int32_t)(sts->req_lon * 1e7);
    float alt = sts->heading_alt_uav;  // Altitude in meters

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
    
    int ret = sendto(sock->sockfd, buf, len, 0, 
                    (struct sockaddr*)&sock->autopilot_addr, 
                    sizeof(sock->autopilot_addr));
    
    if (ret == -1) {
        perror("sendto failed");
    } 

    // Add small delay
   // usleep(100000);  // 100ms delay
}

// Function to send MAVLink SET_ATTITUDE_TARGET
void set_pitch(sockport *sock, sts *sts) {
    uint8_t buf[1024];
    mavlink_message_t msg;
    uint16_t len;

    mavlink_set_attitude_target_t attitude_target = {0};
    attitude_target.time_boot_ms = 0;
    attitude_target.target_system = 1;   // Autopilot system ID
    attitude_target.target_component = 1; // Autopilot component ID
    attitude_target.type_mask = 0b00000111; // Ignore roll, yaw, and body rates
    attitude_target.q[0] = sts->q[0];
    attitude_target.q[1] = sts->q[1];
    attitude_target.q[2] = sts->q[2];
    attitude_target.q[3] = sts->q[3];
    attitude_target.thrust = 0.9;  // 50% thrust

    // Encode message
    mavlink_msg_set_attitude_target_encode(255, 0, &msg, &attitude_target);
    len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send message
    sendto(sock->sockfd, buf, len, 0, (struct sockaddr*)&sock->autopilot_addr, sizeof(sock->autopilot_addr));

    //printf("Sent pitch command: %.2f degrees\n", sts->flight_path_angle);
}

void send_override(joy_s *joy, sockport *sock)
{
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    mavlink_rc_channels_override_t rc_override;
    uint16_t len;
    //joy_s joy;
    //sockport sock;
    //printf("rc: %d, %d, %d, %d\n", joy->roll, joy->pitch, joy->throttle, joy->yaw);

    rc_override.chan1_raw = joy->roll;    // Roll
    rc_override.chan2_raw = joy->pitch;    // Pitch
    rc_override.chan3_raw = joy->throttle;    // Throttle
    rc_override.chan4_raw = joy->yaw;    // Yaw
    rc_override.chan5_raw = 0;
    rc_override.chan6_raw = 0;
    rc_override.chan7_raw = 0;
    rc_override.chan8_raw = 0;
    rc_override.target_system = DEFAULT_TARGET_SYSTEM;
    rc_override.target_component = DEFAULT_TARGET_COMPONENT;

    mavlink_msg_rc_channels_override_encode(GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg, &rc_override);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock->sockfd, buf, len, 0, (struct sockaddr*)&sock->autopilot_addr, sizeof(sock->autopilot_addr));
    //printf("Sent RC_OVERRIDE message\n");
    
}



void *sendautopilot_thread(void *arg)
{
    thread_data_t *thread_data = (thread_data_t *)arg;
    sockport *sock = thread_data->sock;
    sts *sts = thread_data->sts;
    //joy_s *joy = thread_data->joy;
    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;
    while (!stop_flag)
    {
        switch (sts->mission_state) {
            case 0: // Do nothing
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
                sendto(sock->sockfd, buf, len, 0, 
                      (struct sockaddr*)&sock->autopilot_addr, 
                      sizeof(sock->autopilot_addr));
                //printf("Setting Plane GUIDED mode\n");
                reposition(sock, sts);

                    send_mission(sock, sts);

                //sleep(2);
                break;

            case 2: // Send mission
               // for (int n = 0; n < 10; n++) {
                    send_mission(sock, sts);
                    sts->mission_state = 3;
                break;

            case 3: // Set AUTO mode for fixed-wing
                mavlink_msg_command_long_pack(
                    GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
                    DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
                    MAV_CMD_DO_SET_MODE, 0,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    PLANE_MODE_AUTO,
                    0, 0, 0, 0, 0
                );
                len = mavlink_msg_to_send_buffer(buf, &msg);
                sendto(sock->sockfd, buf, len, 0, 
                       (struct sockaddr*)&sock->autopilot_addr, 
                       sizeof(sock->autopilot_addr));
                //printf("Setting Plane AUTO mode\n");
                

               if (sts->t2m_distance < 450)
                    sts->mission_state = 4;
                break;

            case 4: // attack mode start
            mavlink_msg_command_long_pack(
                GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
                DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
                MAV_CMD_DO_SET_MODE, 0,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PLANE_MODE_GUIDED,
                0, 0, 0, 0, 0
            );
            len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock->sockfd, buf, len, 0, 
                   (struct sockaddr*)&sock->autopilot_addr, 
                   sizeof(sock->autopilot_addr));
            set_pitch(sock, sts);
                   //send_override(joy, sock);
                //usleep(10000); // 100 Hz update rate
                break;
        }
        
    }

    printf("Send autopilot thread exited\n");
    return NULL;
}