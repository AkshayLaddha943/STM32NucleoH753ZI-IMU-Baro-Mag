/*
 * imu2quaternion.h
 *
 *  Created on: Jul 26, 2025
 *      Author: ADMIN
 */

 #ifndef INC_IMU2QUATERNION_H_
 #define INC_IMU2QUATERNION_H_
 
 #include "mpu6500.h"
 #include "quaternion.h"
 
 void acc2quat(quaternion *qa, mpu6500_data imu_data);
 
 #endif /* INC_IMU2QUATERNION_H_ */
 