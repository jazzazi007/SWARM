
/*
 * File: gnc.h
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the headers, includings, states UAS,
 *             functions for the guidance, navigation, and control
 *             , and UDP sockets connected with the autopilot.
 *
 */

#ifndef GNC_H
#define GNC_H

#include "../c_library_v2/common/mavlink.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/select.h>
#define M_PI 3.14159265358979323846
#include <pthread.h>
#include <stdatomic.h>
#include <SDL2/SDL.h>

#define UAV_COUNT 1

#define BUFFER_LENGTH 2041
#define DEFAULT_TARGET_SYSTEM 1     // Default to system ID 1 (typical for autopilot)
#define DEFAULT_TARGET_COMPONENT 1  // Default to component ID 1 (flight controller)
#define GCS_SYSTEM_ID 255          // Ground Control Station ID
#define GCS_COMPONENT_ID 0         // GCS component ID

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EARTH_RADIUS 6378137.0
#define MODE_CHANGE_TIMEOUT 500
// Add this at the top with other defines
#define MISSION_RETRY_COUNT 10

// Add this at the top
#define MIN_MISSION_INTERVAL 5     // 50ms minimum interval

// Add this define at the top
#define DO_JUMP_COMMAND 177  // MAV_CMD_DO_JUMP
// Add this function to check for acknowledgments
typedef struct{
//sensors reading from the autopilot
    double gps_lat[UAV_COUNT];
    double gps_lon[UAV_COUNT];
    double gps_alt[UAV_COUNT];

    double heading [UAV_COUNT];
    double airspeed[UAV_COUNT];
    double groundspeed[UAV_COUNT];
    double alt_hud[UAV_COUNT];

    //home position
    double home_lat[UAV_COUNT];
    double home_lon[UAV_COUNT];
    double home_alt[UAV_COUNT];
//end

    double t2m_distance[UAV_COUNT]; //missile to target distance 
    double t2m_altitude[UAV_COUNT]; //missile to target altitude
    double bearing_error[UAV_COUNT]; //bearing error
    double bearing[UAV_COUNT]; //bearing

    //phases
    double req_lat[UAV_COUNT];
    double req_lon[UAV_COUNT];
    double req_alt[UAV_COUNT];
    double t_displacement_x[UAV_COUNT];
    double t_displacement_y[UAV_COUNT];
    double t_lat[UAV_COUNT];
    double t_lon[UAV_COUNT];
    double t_alt[UAV_COUNT];
    double t2m_heading[UAV_COUNT];
    double heading_lat_uav[UAV_COUNT];
    double heading_lon_uav[UAV_COUNT];
    double heading_alt_uav[UAV_COUNT];
    double stable_lat[UAV_COUNT][4];
    double stable_lon[UAV_COUNT][4];
    int stable_alt[UAV_COUNT][4];

    double last_pos_error[3][2];  // Previous position error for each UAV
    double int_error[3][2];       // Integral error for each UAV

    int mission_state; // 0: do nothing, 1: set guided mode, 2: set auto mode, 3: attack mode
} sts;

typedef struct{

    double heading_rate; // roll
    double alt_rate; // pitch
    double distance_rate; // throttle

    double p_gain_heading;
    double p_gain_alt;
    double p_gain_distance;

    double i_gain_heading;
    double i_gain_alt;
    double i_gain_distance;

    double d_gain_heading;
    double d_gain_alt;
    double d_gain_distance;

    double heading_rate_last;
    double alt_rate_last;
    double distance_rate_last;

    double proportional_heading;
    double proportional_alt;
    double proportional_distance;

    double integral_heading;
    double integral_alt;
    double integral_distance;

    double derivative_heading;
    double derivative_alt;
    double derivative_distance;
} gains;

typedef struct {
    uint16_t pitch[UAV_COUNT]; //joy 3
    uint16_t roll[UAV_COUNT]; //joy 2
    uint16_t throttle[UAV_COUNT]; //joy 1
    uint16_t yaw[UAV_COUNT]; //joy 0
} joy_s;

typedef struct{
    mavlink_message_t msg;

    mavlink_heartbeat_t heartbeat_msg;

    mavlink_gps_raw_int_t gps_raw_int;

    mavlink_global_position_int_t global_position_int;
    
    mavlink_vfr_hud_t vfr_hud;

    mavlink_home_position_t home_position;

    mavlink_mission_item_reached_t mission_item_reached;

    mavlink_attitude_t attitude;

}mavlink_str;

typedef struct{
    struct sockaddr_in autopilot_addr[UAV_COUNT];
    uint8_t buf[BUFFER_LENGTH];
    uint16_t len;
    int sockfd[UAV_COUNT];
    int udp_port[UAV_COUNT];
    char *ip_addr[UAV_COUNT];
}sockport;

void read_autopilot(mavlink_str *mavlink_str, sockport *socket, sts *sts, int id);
void send_autopilot(sockport *sock, sts *sts, joy_s *joy, int id);
void coverage_area_triangle(sts *sts, int id);
void rc_init(joy_s *joy, sts *sts, gains *gains);
#endif
