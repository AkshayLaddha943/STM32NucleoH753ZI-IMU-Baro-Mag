#include <stdint.h>
#include <math.h>

#ifndef EST_ATTITUDE_EKF_H
#define EST_ATTITUDE_EKF_H

#define G 9.81

// State vector: [qw, qx, qy, qz, bx, by, bz]
#define STATE_DIMEN 7

// Measurement vector [ax, ay, az]
#define MEASURE_DIMEN 3

typedef struct {
	float x[STATE_DIMEN];
	float P[STATE_DIMEN * STATE_DIMEN];            //Covariance Matrix P
	float Q[STATE_DIMEN * STATE_DIMEN];            //Process Noise Q
	float R[MEASURE_DIMEN * MEASURE_DIMEN];        //Measurement Noise R
	float dt;
}EKF_t;

void ekf_init(EKF_t *ekf, float dt, float g);
void ekf_predict(EKF_t *ekf, float gyro[3]);
void ekf_update(EKF_t *ekf, float accel[3]);
void quaternion_to_euler(float quater[4], float euler[3]);
void omega_matrix(float omega[3], float result[4][4]);
void normalize_quaternion(EKF_t *ekf);
void compute_jacobian(float gyro, EKF_t *ekf);

#endif
