#include "est_orientation_ekf.h"
#include <math.h>

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

}


void ekf_predict(EKF_t *ekf, float gyro[3])
{

}

void ekf_update(EKF_t *ekf, float accel[3])
{

}

void quaternion_to_euler(float quater[4], float euler[3])
{

}


