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

void read_autopilot(mavlink_str *mavlink_str, sockport *socket, sts *sts, int id)
{

    // Send the message
    sendto(socket->sockfd[id], socket->buf, socket->len, 0, (struct sockaddr*)&socket->autopilot_addr[id], sizeof(socket->autopilot_addr[id]));
    //printf("recieved heart beat %s:%d\n", socket->ip_addr, socket->udp_port);

    // Check for incoming data
    struct timeval tv;
    fd_set readfds;

    tv.tv_sec = 1; // 1 second timeout
    tv.tv_usec = 0;

    FD_ZERO(&readfds);
    FD_SET(socket->sockfd[id], &readfds);

    int retval = select(socket->sockfd[id] + 1, &readfds, NULL, NULL, &tv);

    if (retval == -1) {
        perror("select()");
        close(socket->sockfd[id]);
        return ;
    } else if (retval) {
        // Data is available to read
        char recv_buf[BUFFER_LENGTH];
        socklen_t addr_len = sizeof(socket->autopilot_addr[id]);
        int recv_len = recvfrom(socket->sockfd[id], recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&socket->autopilot_addr[id], &addr_len);
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
                        printf("Received HEARTBEAT: Type: %d, Autopilot: %d\n", mavlink_str->heartbeat_msg.type, mavlink_str->heartbeat_msg.autopilot);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
                    {
                        mavlink_msg_gps_raw_int_decode(&mavlink_str->msg, &mavlink_str->gps_raw_int);
                        sts->gps_lat[id] = (double)mavlink_str->gps_raw_int.lat/1e7;
                        sts->gps_lon[id] = (double)mavlink_str->gps_raw_int.lon/1e7;
                        sts->gps_alt[id] = (double)mavlink_str->gps_raw_int.alt/1e3;
                        //sts->gps_vel = (double)mavlink_str->gps_raw_int.vel/1e2;
                       // printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat[id], sts->gps_lon[id]);
                        //printf("Received GPS_RAW_INT: Alt: %f, VD: %f\n", sts->gps_alt, sts->gps_vel);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                    {
                        mavlink_msg_global_position_int_decode(&mavlink_str->msg, &mavlink_str->global_position_int);
                        sts->gps_lat[id] = (double)mavlink_str->global_position_int.lat/1e7;
                        sts->gps_lon[id] = (double)mavlink_str->global_position_int.lon/1e7;
                        sts->gps_alt[id] = (double)mavlink_str->global_position_int.alt/1e3;
                        /*printf("RCV ID: %d\n", id);
                        printf("Received GPS_RAW_INT: Lat: %f, Lon: %f\n", sts->gps_lat[id], sts->gps_lon[id]);
                        printf("Received GPS_RAW_INT: Alt: %f\n", sts->gps_alt[id]);*/
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_VFR_HUD)
                    {
                        mavlink_msg_vfr_hud_decode(&mavlink_str->msg, &mavlink_str->vfr_hud);
                        sts->airspeed[id] = (double)mavlink_str->vfr_hud.airspeed;
                        sts->groundspeed[id] = (double)mavlink_str->vfr_hud.groundspeed;
                        sts->heading[id] = (double)mavlink_str->vfr_hud.heading;
                        sts->alt_hud[id] = (double)mavlink_str->vfr_hud.alt;
                        //printf("Received VFR_HUD: Airspeed: %f, Groundspeed: %f\n", sts->airspeed, sts->groundspeed);
                        //printf("Received VFR_HUD: Heading: %f, Altitude: %f\n", sts->heading, sts->alt_hud);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_HOME_POSITION)
                    {
                        mavlink_msg_home_position_decode(&mavlink_str->msg, &mavlink_str->home_position);
                        sts->home_lat[id] = (double)mavlink_str->home_position.latitude/1e7;
                        sts->home_lon[id] = (double)mavlink_str->home_position.longitude/1e7;
                        sts->home_alt[id] = (double)mavlink_str->home_position.altitude/1e3;
                        //printf("Received HOME_POSITION: ID: %d Lat: %f, Lon: %f\n", id, sts->home_lat[id], sts->home_lon[id]);
                        //printf("Received HOME_POSITION: Alt: %f\n", sts->home_alt);
                    }
                    if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
                    {
                        mavlink_msg_attitude_decode(&mavlink_str->msg, &mavlink_str->attitude);
                        sts->roll[id] = (double)mavlink_str->attitude.roll;
                        sts->pitch[id] = (double)mavlink_str->attitude.pitch;
                        sts->yaw[id] = (double)mavlink_str->attitude.yaw;
                        //printf("Received ATTITUDE: Roll: %f, Pitch: %f, Yaw: %f\n", sts->roll, sts->pitch, sts->yaw);
                    }
                    /*if (mavlink_str->msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
                    {
                        mavlink_msg_mission_item_reached_decode(&mavlink_str->msg, &mavlink_str->mission_item_reached);
                        printf("Received MISSION_ITEM_REACHED: Seq: %d\n", mavlink_str->mission_item_reached.seq);
                    }*/
                }
            }
        }
    } else 
        printf("No data\n");
   // printf("Exiting readautopilot\n");
    //close(socket->sockfd[id]);
    return ;
}