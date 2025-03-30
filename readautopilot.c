/*
 * File: readautopilot.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the implementation of a MAVLink heartbeat receiver.
 *
 */

 #include "../include/swarm.h"

 #define MAX_WAYPOINTS 4

 typedef struct {
     uint16_t count;
     mavlink_mission_item_int_t mission_items[MAX_WAYPOINTS];
     bool receiving;
     uint16_t received;
     uint16_t num_waypoints;
     uint16_t current_mission_seq;
 } mission_data_t;

 static void heart_prep(mavlink_message_t *msg, uint8_t *buf, ssize_t *len)
 {
     mavlink_heartbeat_t heartbeat;
         heartbeat.type = MAV_TYPE_GCS;
         heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
         heartbeat.base_mode = MAV_MODE_MANUAL_ARMED;
         heartbeat.custom_mode = 0;
         heartbeat.system_status = MAV_STATE_ACTIVE;
         mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);
         len = mavlink_msg_to_send_buffer(buf, &msg);
}

void request_mission(sockport *sock)
{
    mavlink_message_t msg;
    uint8_t buf[BUFFER_LENGTH];
    uint16_t len;

    mavlink_msg_mission_request_list_pack(
        GCS_SYSTEM_ID, GCS_COMPONENT_ID, &msg,
        DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT,
        MAV_MISSION_TYPE_MISSION
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock->sockfd, buf, len, 0, 
           (struct sockaddr*)&sock->autopilot_addr, 
           sizeof(sock->autopilot_addr));
}

void readautopilot_thread(mavlink_str *mavlink_str, sockport *socket, sts *sts, int i)
{

    mission_data_t mission_data;
    memset(&mission_data, 0, sizeof(mission_data));
    int count = 0;


    while(!stop_flag){

    heart_prep(&mavlink_str->msg, socket->buf, &socket->len);

    // Send the message
    sendto(socket->sockfd, socket->buf, socket->len, 0, (struct sockaddr*)&socket->autopilot_addr, sizeof(socket->autopilot_addr));
    //printf("recieved heart beat %s:%d\n", socket->ip_addr, socket->udp_port);

    // Check for incoming data
    struct timeval tv;
    fd_set readfds;

    tv.tv_sec = 1; // 1 second timeout
    tv.tv_usec = 0;

    FD_ZERO(&readfds);
    FD_SET(socket->sockfd[i], &readfds);

    int retval = select(socket->sockfd + 1, &readfds, NULL, NULL, &tv);

    if (retval == -1) {
        perror("select()");
        close(socket->sockfd);
        return NULL;
    } else if (retval) {
        // Data is available to read
        char recv_buf[1024];
        socklen_t addr_len = sizeof(socket->autopilot_addr);
        int recv_len = recvfrom(socket->sockfd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&socket->autopilot_addr, &addr_len);
        if (recv_len > 0) {
            // Decode the MAVLink message
            mavlink_status_t status;
            for (int i = 0; i < recv_len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, recv_buf[i], &mavlink_str->msg, &status)) {
                    // Print out the received message details
                   // printf("Received message with ID: %d\n", mavlink_str->msg.msgid);
                    
                    // Process the specific message (e.g., HEARTBEAT)
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_msg_heartbeat_decode(&mavlink_str->msg, &mavlink_str->heartbeat_msg);
                        //printf("Received HEARTBEAT: Type: %d, Autopilot: %d\n", mavlink_str->heartbeat_msg.type, mavlink_str->heartbeat_msg.autopilot);
                    }
                    /*if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
                    {
                        mavlink_msg_gps_raw_int_decode(&mavlink_str->msg, &mavlink_str->gps_raw_int);
                        sts->gps_lat = (double)mavlink_str->gps_raw_int.lat/1e7;
                        sts->gps_lon = (double)mavlink_str->gps_raw_int.lon/1e7;
                        sts->gps_alt = (double)mavlink_str->gps_raw_int.alt/1e3;
                        //sts->gps_vel = (double)mavlink_str->gps_raw_int.vel/1e2;
                        printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat, sts->gps_lon);
                        //printf("Received GPS_RAW_INT: Alt: %f, VD: %f\n", sts->gps_alt, sts->gps_vel);
                    }*/
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                    {
                        mavlink_msg_global_position_int_decode(&mavlink_str->msg, &mavlink_str->global_position_int);
                        sts->gps_lat = (double)mavlink_str->global_position_int.lat/1e7;
                        sts->gps_lon = (double)mavlink_str->global_position_int.lon/1e7;
                        sts->gps_alt = (double)mavlink_str->global_position_int.alt/1e3;
                        //printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat, sts->gps_lon);
                        //printf("Received GPS_RAW_INT: Alt: %f\n", sts->gps_alt);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_VFR_HUD)
                    {
                        mavlink_msg_vfr_hud_decode(&mavlink_str->msg, &mavlink_str->vfr_hud);
                        sts->airspeed = (double)mavlink_str->vfr_hud.airspeed;
                        sts->groundspeed = (double)mavlink_str->vfr_hud.groundspeed;
                        sts->heading = (double)mavlink_str->vfr_hud.heading;
                        sts->alt_hud = (double)mavlink_str->vfr_hud.alt;
                        //printf("Received VFR_HUD: Airspeed: %f, Groundspeed: %f\n", sts->airspeed, sts->groundspeed);
                        //printf("Received VFR_HUD: Heading: %f, Altitude: %f\n", sts->heading, sts->alt_hud);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_HOME_POSITION)
                    {
                        mavlink_msg_home_position_decode(&mavlink_str->msg, &mavlink_str->home_position);
                        sts->home_lat = (double)mavlink_str->home_position.latitude/1e7;
                        sts->home_lon = (double)mavlink_str->home_position.longitude/1e7;
                        sts->home_alt = (double)mavlink_str->home_position.altitude/1e3;
                        printf("Received HOME_POSITION: Lat: %f, Lon: %f\n", sts->home_lat, sts->home_lon);
                        //printf("Received HOME_POSITION: Alt: %f\n", sts->home_alt);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
                    {
                        mavlink_msg_mission_item_reached_decode(&mavlink_str->msg, &mavlink_str->mission_item_reached);
                        printf("Received MISSION_ITEM_REACHED: Seq: %d\n", mavlink_str->mission_item_reached.seq);
                        if (mavlink_str->mission_item_reached.seq == 4)
                            sts->mission_state = 4; // if the mission at its last point this means the attack gnc will start
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
                    {
                        mavlink_msg_attitude_decode(&mavlink_str->msg, &mavlink_str->attitude);
                        sts->attitude_uav[0] = (double)mavlink_str->attitude.pitch * RAD_TO_DEG;
                        sts->attitude_uav[1] = (double)mavlink_str->attitude.roll * RAD_TO_DEG;
                        sts->attitude_uav[2] = (double)mavlink_str->attitude.yaw * RAD_TO_DEG;
                        //printf("Received ATTITUDE: Pitch: %f, Roll: %f\n", sts->attitude_uav[0], sts->attitude_uav[1]);
                        //printf("Received ATTITUDE: Yaw: %f\n", sts->attitude_uav[2]);

                    }
                }
            }
        }
    } else 
        printf("No data\n");
    }
    printf("Exiting readautopilot_thread\n");
    close(socket->sockfd);
    return NULL;
}