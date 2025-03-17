#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

// Constants
const double PI = 3.141592653589793;
const double POSITION_ACCURACY = 0.05;
const double ZETA = 1.1547;
const double ETA = 0.0732;
const double DSTAR = 0.3;
const double QSTAR = 0.75;
const double ERROR_THETA_MAX = PI / 4;  // 45 degrees in radians
const double V_MAX = 0.2;
const double KP_OMEGA = 1.5;
const double OMEGA_MAX = 0.5 * PI;
const double DT = 0.1;
const int T_MAX = 1000;

// Helper functions
double deg2rad(double deg) {
    return deg * PI / 180.0;
}

double norm(double x, double y) {
    return std::sqrt(x * x + y * y);
}

// Main function
int main() {
    // Initial positions and orientations for 3 mass points (forming a V shape)
    std::vector<double> x = {-0.5, -0.5, 4.0};
    std::vector<double> y = {1.0, 0.8, 0.5};
    std::vector<double> theta = {0.0, 0.0, 0.0};

    // Goal position
    double x_goal = 3.5;
    double y_goal = 2.75;

    // V-formation parameters
    double v_distance = 0.3;
    double v_angle = deg2rad(45);

    // Path storage
    std::vector<std::vector<double>> X(3, std::vector<double>(T_MAX));
    std::vector<std::vector<double>> Y(3, std::vector<double>(T_MAX));

    for (int i = 0; i < 3; ++i) {
        X[i][0] = x[i];
        Y[i][0] = y[i];
    }

    // Lyapunov function history
    std::vector<double> V_history(T_MAX, 0.0);

    int t = 0;
    while (t < T_MAX) {
        bool all_reached_goal = true;
        double V_total = 0.0;

        for (int i = 0; i < 3; ++i) {
            if (i == 0) {  // Leader behavior
                double dist_to_goal = norm(x_goal - x[i], y_goal - y[i]);
                if (dist_to_goal > POSITION_ACCURACY) {
                    all_reached_goal = false;

                    // Attractive Potential
                    double nablaU_att_x = 0.0, nablaU_att_y = 0.0;
                    if (dist_to_goal <= DSTAR) {
                        nablaU_att_x = ZETA * (x[i] - x_goal);
                        nablaU_att_y = ZETA * (y[i] - y_goal);
                    } else {
                        double factor = DSTAR / dist_to_goal * ZETA;
                        nablaU_att_x = factor * (x[i] - x_goal);
                        nablaU_att_y = factor * (y[i] - y_goal);
                    }

                    // Repulsive Potential (No obstacles implemented in this simplified version)
                    double nablaU_rep_x = 0.0, nablaU_rep_y = 0.0;

                    // Total Potential
                    double nablaU_x = nablaU_att_x + nablaU_rep_x;
                    double nablaU_y = nablaU_att_y + nablaU_rep_y;

                    // Reference velocity and orientation
                    double theta_ref = atan2(-nablaU_y, -nablaU_x);
                    double error_theta = theta_ref - theta[i];
                    double v_ref = 0.0;

                    if (std::abs(error_theta) <= ERROR_THETA_MAX) {
                        double alpha = (ERROR_THETA_MAX - std::abs(error_theta)) / ERROR_THETA_MAX;
                        v_ref = std::min(alpha * norm(-nablaU_x, -nablaU_y), V_MAX);
                    }

                    double omega_ref = KP_OMEGA * error_theta;
                    omega_ref = std::clamp(omega_ref, -OMEGA_MAX, OMEGA_MAX);

                    // Update kinematic model
                    theta[i] += omega_ref * DT;
                    x[i] += v_ref * std::cos(theta[i]) * DT;
                    y[i] += v_ref * std::sin(theta[i]) * DT;
                }
            } else {  // Followers
                double angle_offset = (i == 1) ? v_angle : -v_angle;
                double formation_x = x[0] - v_distance * std::cos(theta[0] + angle_offset);
                double formation_y = y[0] - v_distance * std::sin(theta[0] + angle_offset);

                double dist_to_formation = norm(formation_x - x[i], formation_y - y[i]);
                if (dist_to_formation > POSITION_ACCURACY) {
                    all_reached_goal = false;

                    // Attractive Potential
                    double nablaU_att_x = 0.0, nablaU_att_y = 0.0;
                    if (dist_to_formation <= DSTAR) {
                        nablaU_att_x = ZETA * (x[i] - formation_x);
                        nablaU_att_y = ZETA * (y[i] - formation_y);
                    } else {
                        double factor = DSTAR / dist_to_formation * ZETA;
                        nablaU_att_x = factor * (x[i] - formation_x);
                        nablaU_att_y = factor * (y[i] - formation_y);
                    }

                    // Total Potential
                    double nablaU_x = nablaU_att_x;
                    double nablaU_y = nablaU_att_y;

                    // Reference velocity and orientation
                    double theta_ref = atan2(-nablaU_y, -nablaU_x);
                    double error_theta = theta_ref - theta[i];
                    double v_ref = 0.0;

                    if (std::abs(error_theta) <= ERROR_THETA_MAX) {
                        double alpha = (ERROR_THETA_MAX - std::abs(error_theta)) / ERROR_THETA_MAX;
                        v_ref = std::min(alpha * norm(-nablaU_x, -nablaU_y), V_MAX);
                    }

                    double omega_ref = KP_OMEGA * error_theta;
                    omega_ref = std::clamp(omega_ref, -OMEGA_MAX, OMEGA_MAX);

                    // Update kinematic model
                    theta[i] += omega_ref * DT;
                    x[i] += v_ref * std::cos(theta[i]) * DT;
                    y[i] += v_ref * std::sin(theta[i]) * DT;
                }
            }
        }

        // Store data
        for (int i = 0; i < 3; ++i) {
            X[i][t] = x[i];
            Y[i][t] = y[i];
        }

        if (all_reached_goal) break;
        t++;
    }

    std::cout << "Travel time: " << (t * DT) << " seconds.\n";
    return 0;
}
