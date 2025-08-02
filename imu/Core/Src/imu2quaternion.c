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
     ax_norm = imu_data.ax;
     ay_norm = imu_data.ay;
     az_norm = imu_data.az;
 
     // quaternion from accelerometer
     qa -> s = (1.0 + az_norm).sqrt()/2.0;
 
 
 
 
 }
 