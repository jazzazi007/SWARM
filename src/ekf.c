#include <stdio.h>
#include <math.h>

#define STATE_DIM 4   // [lat, lon, v_lat, v_lon]
#define MEAS_DIM 2    // [lat, lon]
#define DT 0.1        // Time step

typedef struct {
    double x[STATE_DIM];        // State: [lat, lon, v_lat, v_lon]
    double P[STATE_DIM][STATE_DIM]; // Covariance
} ekf_state;

// Simple matrix utility functions for 4x4 and 2x4, 4x2, 2x2
static void mat4x4_add(double A[4][4], double B[4][4], double C[4][4]) {
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) C[i][j] = A[i][j] + B[i][j];
}
static void mat4x4_sub(double A[4][4], double B[4][4], double C[4][4]) {
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) C[i][j] = A[i][j] - B[i][j];
}
static void mat4x4_mult(double A[4][4], double B[4][4], double C[4][4]) {
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) {
        C[i][j] = 0;
        for (int k=0; k<4; ++k) C[i][j] += A[i][k] * B[k][j];
    }
}
static void mat4x2_mult(double A[4][4], double B[4][2], double C[4][2]) {
    for (int i=0; i<4; ++i) for (int j=0; j<2; ++j) {
        C[i][j] = 0;
        for (int k=0; k<4; ++k) C[i][j] += A[i][k] * B[k][j];
    }
}
static void mat2x4_mult(double A[2][4], double B[4][4], double C[2][4]) {
    for (int i=0; i<2; ++i) for (int j=0; j<4; ++j) {
        C[i][j] = 0;
        for (int k=0; k<4; ++k) C[i][j] += A[i][k] * B[k][j];
    }
}
static void mat2x2_inv(double A[2][2], double invA[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabs(det) < 1e-12) det = 1e-12;
    invA[0][0] =  A[1][1]/det;
    invA[0][1] = -A[0][1]/det;
    invA[1][0] = -A[1][0]/det;
    invA[1][1] =  A[0][0]/det;
}

// EKF predict step
void ekf_predict(ekf_state *ekf) {
    // State transition: x = F*x
    double F[4][4] = {
        {1, 0, DT, 0},
        {0, 1, 0, DT},
        {0, 0, 1, 0 },
        {0, 0, 0, 1 }
    };
    double Q[4][4] = { // Process noise
        {1e-7,0,0,0},
        {0,1e-7,0,0},
        {0,0,1e-8,0},
        {0,0,0,1e-8}
    };

    // x = F*x
    double x_new[4];
    for (int i=0; i<4; ++i) {
        x_new[i] = 0;
        for (int j=0; j<4; ++j)
            x_new[i] += F[i][j] * ekf->x[j];
    }
    for (int i=0; i<4; ++i) ekf->x[i] = x_new[i];

    // P = F*P*F' + Q
    double FP[4][4], FPFt[4][4];
    mat4x4_mult(F, ekf->P, FP);
    double Ft[4][4];
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) Ft[i][j] = F[j][i];
    mat4x4_mult(FP, Ft, FPFt);
    mat4x4_add(FPFt, Q, ekf->P);
}

// EKF update step
void ekf_update(ekf_state *ekf, double meas_lat, double meas_lon) {
    // Measurement matrix
    double H[2][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    };
    double R[2][2] = {
        {1e-6, 0},
        {0, 1e-6}
    };

    // y = z - Hx
    double z[2] = {meas_lat, meas_lon};
    double Hx[2] = {ekf->x[0], ekf->x[1]};
    double y[2] = {z[0] - Hx[0], z[1] - Hx[1]};

    // S = HPH' + R
    double HP[2][4], HPHt[2][2];
    mat2x4_mult(H, ekf->P, HP);
    for (int i=0; i<2; ++i) for (int j=0; j<2; ++j) {
        HPHt[i][j] = 0;
        for (int k=0; k<4; ++k) HPHt[i][j] += HP[i][k] * H[j][k];
        HPHt[i][j] += R[i][j];
    }

    // K = P H' S^-1
    double PHt[4][2];
    for (int i=0; i<4; ++i) for (int j=0; j<2; ++j) {
        PHt[i][j] = 0;
        for (int k=0; k<4; ++k) PHt[i][j] += ekf->P[i][k] * H[j][k];
    }
    double S_inv[2][2];
    mat2x2_inv(HPHt, S_inv);
    double K[4][2];
    for (int i=0; i<4; ++i) for (int j=0; j<2; ++j) {
        K[i][j] = 0;
        for (int k=0; k<2; ++k) K[i][j] += PHt[i][k] * S_inv[k][j];
    }

    // x = x + K*y
    for (int i=0; i<4; ++i)
        for (int j=0; j<2; ++j)
            ekf->x[i] += K[i][j] * y[j];

    // P = (I - K*H)*P
    double KH[4][4] = {0};
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
        for (int k=0; k<2; ++k)
            KH[i][j] += K[i][k] * H[k][j];
    double I_KH[4][4];
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
        I_KH[i][j] = (i==j ? 1.0 : 0.0) - KH[i][j];
    double newP[4][4];
    mat4x4_mult(I_KH, ekf->P, newP);
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
        ekf->P[i][j] = newP[i][j];
}

// Initialize EKF for a UAV id
void ekf_init(ekf_state *ekf, double init_lat, double init_lon) {
    ekf->x[0] = init_lat;
    ekf->x[1] = init_lon;
    ekf->x[2] = 0; // v_lat
    ekf->x[3] = 0; // v_lon
    for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
        ekf->P[i][j] = (i==j) ? 1e-3 : 0;
}

// Example usage
// Call ekf_predict(&ekf[id]); each time step
// Call ekf_update(&ekf[id], gps_lat, gps_lon); when new GPS is received
// The estimated lat/lon is ekf[id].x[0], ekf[id].x[1]