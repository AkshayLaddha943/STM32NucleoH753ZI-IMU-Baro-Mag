#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "est_orientation_ekf.h"
#include "imu2quaternion.h"

void ekf_init(EKF_t *ekf, float dt, float g)
{
	/* State Vector */
	ekf->x[0] = 1.0f;
	ekf->x[1] = 0.0f;
	ekf->x[2] = 0.0f;
	ekf->x[3] = 0.0f;
	ekf->x[4] = 0.0f;
	ekf->x[5] = 0.0f;
	ekf->x[6] = 0.0f;

	/* Covariance Matrix */
	ekf->P[0,0] =  0.0f;
	ekf->P[1,1] =  0.0f;
	ekf->P[2,2] =  0.0f;
	ekf->P[3,3] =  0.0f;
	ekf->P[4,4] =  0.0f;
	ekf->P[5,5] =  0.0f;
	ekf->P[6,6] =  0.0f;

	/* Process Noise Matrix */
	ekf->Q[0,0] =  0.05f;
	ekf->Q[1,1] =  0.05f;
	ekf->Q[2,2] =  0.05f;
	ekf->Q[3,3] =  0.05f;
	ekf->Q[4,4] =  0.01f;
	ekf->Q[5,5] =  0.01f;
	ekf->Q[6,6] =  0.01f;

	/* Measurement Matrix */
	ekf->R[0,0] =  0.02f;
	ekf->R[1,1] =  0.02f;
	ekf->R[2,2] =  0.02f;

}


void ekf_predict(EKF_t *ekf, float gyro[3])
{
/* Dynamics model equation */

	/* 1. Subtract estimated bias from raw gyro measurements (control input) */
	float bias = {ekf -> x[4], ekf->x[5], ekf->x[6]};
	float omega = {gyro[0], gyro[1], gyro[2]} - bias;

	/* 2. Integrate quaternion using ang velocity */
	float q = {ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]};
	float result[4][4];

	float q_dot = 0.5 * omega_matrix * q;
	float q_new = q + q_dot * ekf->t;

	for (int i=0; i<STATE_DIMEN;i++) {
		ekf->x[i] = normalized_q[i];
	};

}

void ekf_update(EKF_t *ekf, float accel[3])
{

}

void quaternion_to_euler(float quater[4], float euler[3])
{

}

void omega_matrix(float omega, float result) {
    result[0][0] = 0.0; result[0][1] = -omega[0]; result[0][2] = -omega[1]; result[0][3] = -omega[2];
    result[1][0] = omega[0]; result[1][1] = 0.0; result[1][2] = omega[2]; result[1][3] = -omega[1];
    result[2][0] = omega[1]; result[2][1] = -omega[2]; result[2][2] = 0.0; result[2][3] = omega[0];
    result[3][0] = omega[2]; result[3][1] = omega[1]; result[3][2] = -omega[0]; result[3][3] = 0.0;
}

void normalize_quaternion(EKF_t *ekf)
{
	float q = {ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]};
	float qnorm = sqrtf(ekf->x[0] * ekf->x[0] + ekf->x[1] * ekf->x[1] + ekf->x[2] * ekf->x[2]
				+ ekf->x[3] * ekf->x[3]);

	if (qnorm > 0.0) {
		float normalized_q = q/qnorm;
		for (int i=0; i<STATE_DIMEN;i++) {
			ekf->x[i] = normalized_q[i];
		};
	};

}

/* Jacobian of motion model */
void compute_f_jacobian(float gyro, EKF_t *ekf)
{
	float q0 = ekf->x[0];
	float q1 = ekf->x[1];
	float q2 = ekf->x[2];
	float q3 = ekf->x[3];
	float bx = ekf->x[4];
	float by = ekf->x[5];
	float bz = ekf->x[6];

	float p = gyro[0] - bx;
	float q = gyro[1] - by;
	float r = gyro[2] - bz;

	float f_jacob[7][7];

	// Quaternion dynamics wrt quaternion
	f_jacob[(0, 1)] = -p * ekf->dt;
	f_jacob[(0, 2)] = -q * ekf->dt;
	f_jacob[(0, 3)] = -r * ekf->dt;

	f_jacob[(1, 0)] =  p * ekf->dt;
	f_jacob[(1, 1)] =  q * ekf->dt;
	f_jacob[(1, 2)] =  r * ekf->dt;

	f_jacob[(2, 0)] =  q * ekf->dt;
	f_jacob[(2, 1)] = -r * ekf->dt;
	f_jacob[(2, 3)] =  p * ekf->dt;

	f_jacob[(3, 0)] =  r * ekf->dt;
	f_jacob[(3, 1)] =  q * ekf->dt;
	f_jacob[(3, 2)] = -p * ekf->dt;

	// Quaternion dynamics wrt bias
	f_jacob[(0, 4)] =  0.5 * q1 * ekf->dt;
	f_jacob[(0, 5)] =  0.5 * q2 * ekf->dt;
	f_jacob[(0, 6)] =  0.5 * q3 * ekf->dt;

	f_jacob[(1, 4)] = -0.5 * q0 * ekf->dt;
	f_jacob[(1, 5)] = -0.5 * q3 * ekf->dt;
	f_jacob[(1, 6)] =  0.5 * q2 * ekf->dt;

	f_jacob[(2, 4)] =  0.5 * q3 * ekf->dt;
	f_jacob[(2, 5)] = -0.5 * q0 * ekf->dt;
	f_jacob[(2, 6)] = -0.5 * q1 * ekf->dt;

	f_jacob[(3, 4)] = -0.5 * q2 * ekf->dt;
	f_jacob[(3, 5)] =  0.5 * q1 * ekf->dt;
	f_jacob[(3, 6)] = -0.5 * q0 * ekf->dt;

}
