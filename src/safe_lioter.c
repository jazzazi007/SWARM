#include "../include/swarm.h"

void safe_lioter(sts *sts)
{
    int init_loiter = 150;
    int distance = 30;
    int lioter_0 = (UAV_COUNT-1)/2;
    sts->loiter[0] = 150 + (distance * lioter_0);
    int turn = 0;
    bool check = false;
    for (int i = 1; i < UAV_COUNT; i++)
    {
            sts->loiter[i] = init_loiter + (distance * turn);
            if (i+1 > UAV_COUNT/2 && !check)
            {
                check = true;
                turn+=2;
            }
              
            else
                turn+=1;
    }
}

void safe_alt(sts *sts)
{
    int init_alt = 100;
    int distance = 10;
    int alt_0 = (UAV_COUNT-1)/2;
    sts->stable_alt[0][0] = 100 + (distance * alt_0);
    int turn = 0;
    bool check = false;
    for (int i = 1; i < UAV_COUNT; i++)
    {
            sts->stable_alt[i][0] = init_alt + (distance * turn);
            if (i+1 > UAV_COUNT/2 && !check)
            {
                check = true;
                turn+=2;
            }
              
            else
                turn+=1;
    }
}

void extreme_turn(sts *sts)
{
    float roll_angle = 0;
    // lambda LOS = bearing error from sts
    float g = 9.81; //m/s^2
    //change adaptive gain with variable of k_gain
    //roll_angle = atan2(sts->airspeed * adaptive_gain(sts) * (sts->bearing_error * DEG_TO_RAD), g) * RAD_TO_DEG;
    roll_angle = atan2(sts->airspeed[0] * 0.8 * (sts->bearing_error[0] * DEG_TO_RAD), g) * RAD_TO_DEG;

    //roll_angle = roll_angle + 0.8 * sts->bearing_error;
    if (roll_angle > 20)
        roll_angle = 20;
    else if (roll_angle < -20)
        roll_angle = -20;
    sts->desired_roll_angle[0] = roll_angle;
    // sts->desired_roll_angle = sts->desired_roll_angle + 0.8 *sts->yaw_err_angle;
}


//part of long
#include <stdio.h>
#include <math.h>

#define N 5

// Assign velocity for the leader (agent 0) using consensus law
double assign_leader_speed(
    int n,                // number of agents
    double kappa,         // consensus gain
    double V_d,           // desired average speed
    double R_l,           // leader orbit radius
    double eta_delayed[], // delayed bearing angles [rad]
    int A[][N]            // adjacency matrix
) {
    double consensus_sum = 0.0;
    for (int j = 0; j < n; ++j) {
        consensus_sum += A[0][j] * (eta_delayed[0] - eta_delayed[j]);
    }
    double V_leader = V_d + kappa * R_l * consensus_sum;
    return V_leader;
}
/*
// Example usage
int main() {
    double kappa = 0.5;
    double V_d = 20.0;
    double R_l = 200.0;
    double eta_delayed[N] = {M_PI/3, 55.0*M_PI/180, 58.0*M_PI/180, 53.0*M_PI/180, 50.0*M_PI/180};
    int A[N][N] = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {0, 0, 0, 1, 0}
    };

    double V_leader = assign_leader_speed(N, kappa, V_d, R_l, eta_delayed, A);
    printf("Leader: Assigned Velocity = %.2f m/s\n", V_leader);
    return 0;
}*/
#include <math.h>
#include <stddef.h>

#define DIM 3  // 3D space

// p: positions of all drones [num_drones][3]
// p_goal: goal positions for all drones [num_drones][3]
// num_drones: number of drones
// i: index of the current drone
// k_att: attractive gain
// k_rep: repulsive gain
// d0: repulsive influence distance
// F_out: output force vector [3] for drone i
void apf_3d(
    double **p, double **p_goal, int num_drones, int i,
    double k_att, double k_rep, double d0,
    double *F_out
) {
    // Attractive force
    for (int d = 0; d < DIM; ++d)
        F_out[d] = -k_att * (p[i][d] - p_goal[i][d]);

    // Repulsive force from other drones
    for (int j = 0; j < num_drones; ++j) {
        if (j == i) continue;
        double diff[DIM];
        double dist = 0.0;
        for (int d = 0; d < DIM; ++d) {
            diff[d] = p[i][d] - p[j][d];
            dist += diff[d] * diff[d];
        }
        dist = sqrt(dist);
        if (dist < d0 && dist > 1e-6) {
            double scale = k_rep * (1.0/dist - 1.0/d0) / (dist*dist);
            for (int d = 0; d < DIM; ++d)
                F_out[d] += scale * diff[d];
        }
    }
}

void v_recovery(sts *sts)
{

}
