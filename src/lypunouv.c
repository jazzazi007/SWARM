#ifndef LYPUNOUV_H
#define LYPUNOUV_H

#include "swarm.h"

// Lyapunov control functions
void update_position_lyapunov(sts *sts, int id);
void run_lyapunov(sts *sts);

#endif // LYPUNOUV_H

#include "../include/swarm.h"
#include <math.h>

#define K_P 1.0     // Proportional gain
#define K_D 0.5     // Derivative gain
#define K_I 0.1     // Integral gain
#define DT 0.1      // Time step in seconds

// Lyapunov function calculation
static double calculate_lyapunov(sts *sts, int id) {
    double pos_error[2], vel_error[2];
    
    // Position error (desired - current)
    pos_error[0] = sts->req_lat[id] - sts->gps_lat[id];
    pos_error[1] = sts->req_lon[id] - sts->gps_lon[id];
    
    // Velocity error (desired - current)
    vel_error[0] = (pos_error[0] - sts->last_pos_error[id][0]) / DT;
    vel_error[1] = (pos_error[1] - sts->last_pos_error[id][1]) / DT;
    
    // Lyapunov function V = 0.5*(pos_error^2 + vel_error^2)
    return 0.5 * (pos_error[0]*pos_error[0] + pos_error[1]*pos_error[1] +
                  vel_error[0]*vel_error[0] + vel_error[1]*vel_error[1]);
}

// Control law calculation
static void calculate_control(sts *sts, int id, double *u_lat, double *u_lon) {
    double pos_error[2], vel_error[2], int_error[2];
    
    // Position error
    pos_error[0] = sts->req_lat[id] - sts->gps_lat[id];
    pos_error[1] = sts->req_lon[id] - sts->gps_lon[id];
    
    // Velocity error
    vel_error[0] = (pos_error[0] - sts->last_pos_error[id][0]) / DT;
    vel_error[1] = (pos_error[1] - sts->last_pos_error[id][1]) / DT;
    
    // Integral error
    sts->int_error[id][0] += pos_error[0] * DT;
    sts->int_error[id][1] += pos_error[1] * DT;
    int_error[0] = sts->int_error[id][0];
    int_error[1] = sts->int_error[id][1];
    
    // Control law u = -K_P*pos_error - K_D*vel_error - K_I*int_error
    *u_lat = -K_P * pos_error[0] - K_D * vel_error[0] - K_I * int_error[0];
    *u_lon = -K_P * pos_error[1] - K_D * vel_error[1] - K_I * int_error[1];
    
    // Store current error for next iteration
    sts->last_pos_error[id][0] = pos_error[0];
    sts->last_pos_error[id][1] = pos_error[1];
}

// Update UAV position using Lyapunov-based control
void update_position_lyapunov(sts *sts, int id) {
    double u_lat, u_lon;
    double v = calculate_lyapunov(sts, id);
    
    // Calculate control inputs
    calculate_control(sts, id, &u_lat, &u_lon);
    
    // Update position based on control inputs
    double scale = meters_to_degrees(DT, sts->gps_lat[id]);
    sts->req_lat[id] += u_lat * scale;
    sts->req_lon[id] += u_lon * scale;
    
    // Keep altitude constant
    sts->req_alt[id] = sts->gps_alt[id];
    
    printf("UAV %d: Lyapunov=%.4f, Control(lat,lon)=(%.4f,%.4f)\n",
           id, v, u_lat, u_lon);
}

// Main Lyapunov control function
void run_lyapunov(sts *sts) {
    // Initialize error storage if not already done
    static int initialized = 0;
    if (!initialized) {
        for (int i = 0; i < 3; i++) {
            sts->last_pos_error[i][0] = 0;
            sts->last_pos_error[i][1] = 0;
            sts->int_error[i][0] = 0;
            sts->int_error[i][1] = 0;
        }
        initialized = 1;
    }
    
    // Update positions for all UAVs
    for (int id = 0; id < 3; id++) {
        update_position_lyapunov(sts, id);
    }
}