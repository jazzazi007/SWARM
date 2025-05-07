#ifndef PSO_H
#define PSO_H

#include "../include/swarm.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define NUM_PARTICLES 20
#define NUM_DIMENSIONS 6  // 2 coordinates (lat, lon) for each of 3 UAVs
#define MAX_ITERATIONS 100
#define W 0.729       // Inertia weight
#define C1 2.05       // Cognitive coefficient
#define C2 2.05       // Social coefficient
#define MAX_POSITION_CHANGE 0.001  // Maximum position change in degrees (~100m)
#define MAX_VELOCITY 0.0001       // Maximum velocity in degrees/iteration

typedef struct {
    double position[NUM_DIMENSIONS];    // Current position
    double velocity[NUM_DIMENSIONS];    // Current velocity
    double pbest[NUM_DIMENSIONS];       // Personal best position
    double pbest_val;                  // Personal best value
} Particle;

// Core PSO functions
void init_particles(Particle *particles, sts *sts);
double evaluate_particle(double *position, sts *sts);
void update_particles(Particle *particles, double *gbest, sts *sts);
void run_pso(sts *sts);

#endif // PSO_H

#include <math.h>
#include <stdlib.h>
#include <time.h>

static double gbest[NUM_DIMENSIONS];    // Global best position
static double gbest_val;               // Global best value

// Initialize particles with random positions around current UAV positions
void init_particles(Particle *particles, sts *sts) {
    srand(time(NULL));
    
    for (int i = 0; i < NUM_PARTICLES; i++) {
        for (int j = 0; j < NUM_DIMENSIONS; j += 2) {
            // Get UAV index (0, 1, or 2)
            int uav_idx = j / 2;
            
            // Initialize position with random offset from current position
            double rand_offset = ((double)rand() / RAND_MAX - 0.5) * 0.001; // ±0.001 degrees
            particles[i].position[j] = sts->gps_lat[uav_idx] + rand_offset;
            particles[i].position[j+1] = sts->gps_lon[uav_idx] + rand_offset;
            
            // Initialize velocity
            particles[i].velocity[j] = 0.0;
            particles[i].velocity[j+1] = 0.0;
            
            // Initialize personal best
            particles[i].pbest[j] = particles[i].position[j];
            particles[i].pbest[j+1] = particles[i].position[j+1];
        }
        
        // Evaluate particle
        particles[i].pbest_val = evaluate_particle(particles[i].position, sts);
        
        // Update global best if needed
        if (i == 0 || particles[i].pbest_val > gbest_val) {
            gbest_val = particles[i].pbest_val;
            for (int j = 0; j < NUM_DIMENSIONS; j++) {
                gbest[j] = particles[i].position[j];
            }
        }
    }
}

// Evaluate particle fitness (objective function)
double evaluate_particle(double *position, sts *sts) {
    double fitness = 0.0;
    
    // Calculate triangle area formed by UAVs
    double area = fabs((position[0] - position[4]) * (position[3] - position[5]) -
                      (position[2] - position[4]) * (position[1] - position[5])) / 2.0;
    
    // Add area to fitness
    fitness += area;
    
    // Subtract penalties for constraints
    // Example: Maintain minimum distance between UAVs
    double min_distance = 0.001; // About 100 meters in degrees
    for (int i = 0; i < NUM_DIMENSIONS; i += 2) {
        for (int j = i + 2; j < NUM_DIMENSIONS; j += 2) {
            double dx = position[i] - position[j];
            double dy = position[i+1] - position[j+1];
            double dist = sqrt(dx*dx + dy*dy);
            if (dist < min_distance) {
                fitness -= (min_distance - dist) * 1000;
            }
        }
    }
    
    return fitness;
}

// Update particle positions and velocities
void update_particles(Particle *particles, double *gbest, sts *sts) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        for (int j = 0; j < NUM_DIMENSIONS; j++) {
            // Random coefficients
            double r1 = (double)rand() / RAND_MAX;
            double r2 = (double)rand() / RAND_MAX;
            
            // Update velocity with constraints
            particles[i].velocity[j] = W * particles[i].velocity[j] +
                                     C1 * r1 * (particles[i].pbest[j] - particles[i].position[j]) +
                                     C2 * r2 * (gbest[j] - particles[i].position[j]);
            
            // Clamp velocity
            if (particles[i].velocity[j] > MAX_VELOCITY)
                particles[i].velocity[j] = MAX_VELOCITY;
            if (particles[i].velocity[j] < -MAX_VELOCITY)
                particles[i].velocity[j] = -MAX_VELOCITY;
            
            // Update position
            double new_pos = particles[i].position[j] + particles[i].velocity[j];
            
            // Constrain position change relative to initial position
            int uav_idx = j / 2;
            double ref_pos = (j % 2 == 0) ? sts->gps_lat[uav_idx] : sts->gps_lon[uav_idx];
            if (fabs(new_pos - ref_pos) > MAX_POSITION_CHANGE) {
                new_pos = ref_pos + (new_pos > ref_pos ? MAX_POSITION_CHANGE : -MAX_POSITION_CHANGE);
            }
            
            particles[i].position[j] = new_pos;
        }
        
        // Evaluate new position
        double fitness = evaluate_particle(particles[i].position, sts);
        
        // Update personal best if needed
        if (fitness > particles[i].pbest_val) {
            particles[i].pbest_val = fitness;
            for (int j = 0; j < NUM_DIMENSIONS; j++) {
                particles[i].pbest[j] = particles[i].position[j];
            }
            
            // Update global best if needed
            if (fitness > gbest_val) {
                gbest_val = fitness;
                for (int j = 0; j < NUM_DIMENSIONS; j++) {
                    gbest[j] = particles[i].position[j];
                }
            }
        }
    }
}

// Main PSO algorithm
void run_pso(sts *sts) {
    Particle particles[NUM_PARTICLES];
    
    // Initialize particles
    init_particles(particles, sts);
    
    // Main PSO loop
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        update_particles(particles, gbest, sts);
        
        // Print progress
        printf("Iteration %d: Best fitness = %f\n", iter, gbest_val);
        printf("Best positions:\n");
        for (int i = 0; i < 3; i++) {
            printf("UAV %d: Lat = %f, Lon = %f\n", 
                   i, gbest[i*2], gbest[i*2+1]);
        }
    }
    
    // Update target positions in sts structure
    for (int i = 0; i < 3; i++) {
        sts->req_lat[i] = gbest[i*2];
        sts->req_lon[i] = gbest[i*2+1];
    }
}
int main() {
    // Create and initialize sts structure
    sts test_sts;
    
    // Initialize leader (UAV 0) position - Center position
    test_sts.gps_lat[0] = -35.3627010;
    test_sts.gps_lon[0] = 149.1652270;
    test_sts.gps_alt[0] = 100.0;

    // Initialize follower 1 position - Slightly offset
    test_sts.gps_lat[1] = -35.3627010 + 0.0001; // ~10m North
    test_sts.gps_lon[1] = 149.1652270 + 0.0001; // ~10m East
    test_sts.gps_alt[1] = 100.0;

    // Initialize follower 2 position - Slightly offset
    test_sts.gps_lat[2] = -35.3627010 - 0.0001; // ~10m South
    test_sts.gps_lon[2] = 149.1652270 - 0.0001; // ~10m West
    test_sts.gps_alt[2] = 100.0;

    // Copy initial positions to requested positions
    for (int i = 0; i < 3; i++) {
        test_sts.req_lat[i] = test_sts.gps_lat[i];
        test_sts.req_lon[i] = test_sts.gps_lon[i];
        test_sts.req_alt[i] = test_sts.gps_alt[i];
    }

    printf("Initial UAV Positions:\n");
    for (int i = 0; i < 3; i++) {
        printf("UAV %d: Lat = %.7f, Lon = %.7f\n", 
               i, test_sts.gps_lat[i], test_sts.gps_lon[i]);
    }

    // Run PSO algorithm
    printf("\nRunning PSO optimization...\n\n");
    run_pso(&test_sts);

    printf("\nOptimized UAV Positions:\n");
    for (int i = 0; i < 3; i++) {
        printf("UAV %d: Lat = %.7f, Lon = %.7f\n", 
               i, test_sts.req_lat[i], test_sts.req_lon[i]);
    }

    return 0;
}