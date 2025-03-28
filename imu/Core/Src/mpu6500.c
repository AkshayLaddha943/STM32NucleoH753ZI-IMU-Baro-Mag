#include <mpu6500.h>
#include <main.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
int8_t i2c_read_flag = 0;
float ax, ay, az;

void mpu6500_init()
{
	 HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS <<1) + 0, 1, 100);
	  if (ret == HAL_OK)
	  {
		  printf("The device is ready \n");
	  }
	  else
	  {
		  printf("The device is not ready. CHeck cables \n ");
	  }

	  uint8_t temp_data = FS_GYRO_500;

	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);

	  if (ret == HAL_OK) {
		  printf( "Writing to register 27 \n");
	  }
	  else {
		  printf("No registers found");
	  }

	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK)
	    {
	  	  printf("Configuring gyroscope \n");
	    }
	    else
	    {
	  	  printf("Failed to configure gyroscope \n ");
	    }

	  temp_data = FS_ACC_4G;

	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK)
		{
		  printf("Configuring accelerometer \n");
		}
		else
		{
		  printf("Failed to configure the accelerometer  \n ");
		}

	  uint8_t wake_cmd = 0x00;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_USR_CTRL, 1, &wake_cmd, 1, 100);
	  if (ret == HAL_OK)
		{
		  printf("Exiting from sleep mode \n");
		}
		else
		{
		  printf("Failed to exit from sleep mode \n ");
		}

//	  temp_data = 0;
}

void mpu6500_read()
{
	uint8_t data[21];
	int16_t x_acc, y_acc, z_acc, temp, x_gyro, y_gyro, z_gyro;

	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS <<1), REG_DATA, 1, data, 22, 100);

	x_acc = (data[0] << 8)| data[1];
	y_acc = (data[2] << 8) | data[3];
	z_acc = (data[4] << 8) | data[5];

	temp = (data[6] << 8) | data[7];
	x_gyro = (data[8] << 8) | data[9];
	y_gyro = (data[10] << 8) | data[11];
	z_gyro = (data[12] << 8) | data[13];

	ax = x_acc / 8192.0;
	ay = y_acc / 8192.0;
	az = z_acc / 8192.0;

	float gx = x_gyro / 65.5;
	float gy = y_gyro / 65.5;
	float gz = z_gyro / 65.5;

	float temp_data = (temp / 333.87) + 21.0;

	printf("x axis acceleration: %.f g\n", ax);
	printf("y axis acceleration: %.2f g\n", ay);
	printf("z axis acceleration: %.2f g\n", az);

	printf("Temperature: %.2f °C\n", temp_data);

	printf("x axis rotation: %.2f °/s\n", gx);
	printf("y axis rotation: %.2f °/s\n", gy);
	printf("z axis rotation: %.2f °/s\n", gz);

//	printf("x axis heading: %.2f °/s\n", x_mag);
//	printf("y axis heading: %.2f °/s\n", y_mag);
//	printf("z axis heading: %.2f °/s\n", z_mag);

	printf("\n");

//	data_imu -> y_acc = ((int16_t)data2[2] << 8) + data2[3];
//	data_imu -> z_acc = ((int16_t)data2[4] << 8) + data2[5];
//
//	data_imu -> temp = ((int16_t)data2[6] << 8) + data2[7];
//	data_imu -> x_gyro = ((int16_t)data2[8] << 8) + data2[9];
//	data_imu -> y_gyro = ((int16_t)data2[10] << 8) + data2[11];
//	data_imu -> z_gyro = ((int16_t)data2[12] << 8) + data2[13];
}

