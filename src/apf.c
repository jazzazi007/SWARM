#ifndef APF_H
#define APF_H

#include "swarm.h"

// APF Parameters
#define ATTRACT_GAIN 1.0
#define REPEL_GAIN 100.0
#define INFLUENCE_DIST 200.0  // meters
#define MIN_DIST 50.0        // minimum safe distance between UAVs

// APF functions
void calculate_apf_forces(sts *sts, int id, double *force_x, double *force_y);
void update_position_apf(sts *sts, int id);
void run_apf(sts *sts);

#endif // APF_H

#include <math.h>

// Convert meters to degrees (approximate)
static double meters_to_degrees(double meters, double lat) {
    return meters / (EARTH_RADIUS * cos(lat * DEG_TO_RAD)) * RAD_TO_DEG;
}

// Calculate distance between two points in meters
static double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

// Calculate attractive and repulsive forces for a UAV
void calculate_apf_forces(sts *sts, int id, double *force_x, double *force_y) {
    *force_x = 0;
    *force_y = 0;
    
    // Calculate attractive force to target position
    double dist_to_target = calculate_distance(
        sts->gps_lat[id], sts->gps_lon[id],
        sts->req_lat[id], sts->req_lon[id]
    );
    
    if (dist_to_target > 0.1) {  // Only if not very close to target
        double attract_magnitude = ATTRACT_GAIN * dist_to_target;
        double dx = sts->req_lat[id] - sts->gps_lat[id];
        double dy = sts->req_lon[id] - sts->gps_lon[id];
        double norm = sqrt(dx*dx + dy*dy);
        
        *force_x += attract_magnitude * dx / norm;
        *force_y += attract_magnitude * dy / norm;
    }
    
    // Calculate repulsive forces from other UAVs
    for (int i = 0; i < 3; i++) {
        if (i != id) {
            double dist = calculate_distance(
                sts->gps_lat[id], sts->gps_lon[id],
                sts->gps_lat[i], sts->gps_lon[i]
            );
            
            if (dist < INFLUENCE_DIST) {
                double repel_magnitude = REPEL_GAIN * (1.0/dist - 1.0/INFLUENCE_DIST);
                if (dist < MIN_DIST) {
                    repel_magnitude *= 10.0;  // Stronger repulsion when too close
                }
                
                double dx = sts->gps_lat[id] - sts->gps_lat[i];
                double dy = sts->gps_lon[id] - sts->gps_lon[i];
                double norm = sqrt(dx*dx + dy*dy);
                
                *force_x += repel_magnitude * dx / norm;
                *force_y += repel_magnitude * dy / norm;
            }
        }
    }
}

// Update UAV position based on APF forces
void update_position_apf(sts *sts, int id) {
    double force_x, force_y;
    calculate_apf_forces(sts, id, &force_x, &force_y);
    
    // Convert forces to position updates (scaled by time step)
    double dt = 0.1;  // Time step in seconds
    double scale = meters_to_degrees(dt, sts->gps_lat[id]);
    
    // Update requested position
    sts->req_lat[id] = sts->gps_lat[id] + force_x * scale;
    sts->req_lon[id] = sts->gps_lon[id] + force_y * scale;
    
    // Keep altitude constant
    sts->req_alt[id] = sts->gps_alt[id];
    
    printf("UAV %d: Force (x,y)=(%.2f,%.2f), New pos=(%.6f,%.6f)\n",
           id, force_x, force_y, sts->req_lat[id], sts->req_lon[id]);
}

// Main APF algorithm
void run_apf(sts *sts) {
    // Update positions for all UAVs
    for (int id = 0; id < 3; id++) {
        update_position_apf(sts, id);
    }
}