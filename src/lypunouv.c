#include "../include/swarm.h"
#include <math.h>
#include <stdio.h>
#include <time.h>

#define K_P 1.0     // Proportional gain
#define K_D 0.5     // Derivative gain
#define K_I 0.1     // Integral gain
#define DT 0.1      // Time step in seconds

// Add these structures for data collection
typedef struct {
    double time;
    double lyapunov_value;
    double pos_error[2];
    double vel_error[2];
    double control_inputs[2];
} stability_data;

// Add this function to collect stability data
static void log_stability_data(int id, double v, double *pos_error, double *vel_error, 
                               double u_lat, double u_lon) {
    static FILE *fps[3] = {NULL, NULL, NULL}; // One file pointer for each UAV
    static double last_write_time[3] = {0, 0, 0}; // Track last write time for each UAV

    // Validate UAV ID
    if (id < 0 || id >= 3) {
        fprintf(stderr, "Invalid UAV ID: %d\n", id);
        return;
    }

    // Open a separate file for each UAV
    if (!fps[id]) {
        char filename[32];
        snprintf(filename, sizeof(filename), "stability_data_uav%d.csv", id);
        fps[id] = fopen(filename, "w");
        if (!fps[id]) {
            perror("Failed to open file");
            return;
        }
        // Write the header to the file
        fprintf(fps[id], "Time,UAV_ID,Lyapunov,PosError_Lat,PosError_Lon,VelError_Lat,VelError_Lon,Control_Lat,Control_Lon\n");
    }

    // Validate input pointers
    if (!pos_error || !vel_error) {
        fprintf(stderr, "Null pointer passed to log_stability_data\n");
        return;
    }

    // Get the current time
    static double start_time = 0;
    if (start_time == 0) {
        start_time = (double)clock() / CLOCKS_PER_SEC;
    }
    double current_time = (double)clock() / CLOCKS_PER_SEC - start_time;

    // Write to the file only if 0.25 seconds have passed since the last write
    if (current_time - last_write_time[id] >= 0.25) {
        fprintf(fps[id], "%.3f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                current_time, id, v,
                pos_error[0], pos_error[1],
                vel_error[0], vel_error[1],
                u_lat, u_lon);
        fflush(fps[id]); // Ensure data is written to the file
        last_write_time[id] = current_time;
    }
}

void close_stability_logs(void) {
    static FILE *fps[3] = {NULL, NULL, NULL};
    for (int i = 0; i < 3; i++) {
        if (fps[i]) {
            fclose(fps[i]);
            fps[i] = NULL;
        }
    }
}

// Lyapunov function calculation
static double calculate_lyapunov(sts *sts, int id) {
    double pos_error[2];

    // Position error (desired - current)
    pos_error[0] = sts->req_lat[id] - sts->gps_lat[id];
    pos_error[1] = sts->req_lon[id] - sts->gps_lon[id];

    // Lyapunov function V = 0.5 * (pos_error^2)
    return 0.5 * (pos_error[0] * pos_error[0] + pos_error[1] * pos_error[1]);
}

// Control law calculation
static void calculate_control(sts *sts, int id, double *u_lat, double *u_lon) {
    double pos_error[2];

    // Position error
    pos_error[0] = sts->req_lat[id] - sts->gps_lat[id];
    pos_error[1] = sts->req_lon[id] - sts->gps_lon[id];

    // Control law u = -K_P * pos_error
    *u_lat = -K_P * pos_error[0];
    *u_lon = -K_P * pos_error[1];
}

static double meters_to_degrees(double meters, double latitude) {
    // Earth's radius in meters
    
    
    // Convert latitude to radians
    double lat_rad = latitude * DEG_TO_RAD;
    
    // For latitude: 1 degree = 111,111 meters (approximately)
    // For longitude: 1 degree = 111,111 * cos(latitude) meters
    // We'll use an average conversion since we're dealing with small distances
    double meters_per_degree = EARTH_RADIUS * DEG_TO_RAD * cos(lat_rad);
    
    return meters / meters_per_degree;
}

// Update UAV position using Lyapunov-based control
void update_position_lyapunov(sts *sts, int id) {
    double u_lat, u_lon;

    // Calculate control inputs
    calculate_control(sts, id, &u_lat, &u_lon);

    // Update position based on control inputs
    double scale = meters_to_degrees(DT, sts->gps_lat[id]);
    sts->req_lat[id] += u_lat * scale;
    sts->req_lon[id] += u_lon * scale;

    // Keep altitude constant
    sts->req_alt[id] = sts->gps_alt[id];

    // Calculate errors
    double pos_error[2] = {sts->req_lat[id] - sts->gps_lat[id], sts->req_lon[id] - sts->gps_lon[id]};
    double vel_error[2] = {0, 0}; // Replace with actual velocity error if available

    // Log stability data for this UAV
    double v = calculate_lyapunov(sts, id);
    log_stability_data(id, v, pos_error, vel_error, u_lat, u_lon);
}

// Main Lyapunov control function
void run_lyapunov(sts *sts) {
    // Initialize error storage if not already done
    sts->req_lat[0] = -35.3708726;
    sts->req_lon[0] = 149.1726422;
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