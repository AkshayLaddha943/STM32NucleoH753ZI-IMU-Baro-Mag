/*
 * imu2quaternion.c
 *
 *  Created on: Jul 26, 2025
 *      Author: ADMIN
 */

 #include "math.h"
 #include "stdio.h"
 #include "quaternion.h"
 #include "mpu6500.h"
 #include "imu2quaternion.h"
 
 
 void acc2quat(quaternion *qa, mpu6500_data imu_data)
 {
     // Normalize accelerometer data
     float ax_norm = imu_data.ax;
     float ay_norm = imu_data.ay;
     float az_norm = imu_data.az;
 
     // quaternion from accelerometer
     qa -> s = (1.0 + az_norm).sqrt()/2.0;
     qa -> x = -ay_norm/(2.0 * s);
     qa -> y = ax_norm/(2.0 * s);
     qa -> z = 0.0f;
 
     float qnorm = sqrtf(quat->s * quat->s + quat->x * quat->x + quat->y * quat->y
             + quat->z * quat->z);
     qa -> s /= qnorm;
     qa -> x /= qnorm;
     qa -> y /= qnorm;
     qa -> z /= qnorm;
 
 
 }
 