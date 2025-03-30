
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
#include <SDL2/SDL.h>
#include <math.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/select.h>
#define M_PI 3.14159265358979323846
#include <pthread.h>
#include <stdatomic.h>

#define WINDOW_WIDTH 1600
#define WINDOW_HEIGHT 1200
#define POINT_RADIUS 5
#define TRAJECTORY_RADIUS 100
#define SPEED 0.0005

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

#define GUIDED_2_ATTACK 1

// Add this function to check for acknowledgments


extern atomic_int stop_flag;

typedef struct{
//sensors reading from the autopilot
    double gps_lat;
    double gps_lon;
    double gps_alt;

    double heading;
    double airspeed;
    double groundspeed;
    double alt_hud;

    //home position
    double home_lat;
    double home_lon;
    double home_alt;
//end

    double t2m_distance; //missile to target distance 
    double t2m_altitude;
    double bearing_error;
    double bearing;

    //phases
    double req_lat;
    double req_lon;
    double req_alt;
    double t_displacement_x;
    double t_displacement_y;
    double t_lat;
    double t_lon;
    double t_alt;
    double t2m_heading;
    double heading_lat_uav;
    double heading_lon_uav;
    double heading_alt_uav;
    double stable_lat[4];
    double stable_lon[4];
    int stable_alt[4];
    //LOS
    double los_angle;
    double last_los_angle;
    //param
    double last_pitch_angle;
    float N_gain;
    float k_gain;

    double flight_path_angle; //desired flight path angle
    double last_flight_path_angle; // achieved flight path angle
    float q[4]; // quaternion
    double desired_roll_angle;

    int mission_state;

    double attitude_uav[3]; //pitch, roll, yaw
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
    int pitch; //joy 3
    int roll; //joy 2
    int throttle; //joy 1
    int yaw; //joy 0
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
    struct sockaddr_in autopilot_addr[3];
    uint8_t buf[2041];
    ssize_t len;
    int sockfd[3];
    int udp_port[3];
    char *ip_addr[3];
}sockport;



void send_override();
void readautopilot_thread(mavlink_str *mavlink_str, sockport *socket, sts *sts, int i);
void send_autopilot(sockport *sock, sts *sts, joy_s *joy);





#endif
