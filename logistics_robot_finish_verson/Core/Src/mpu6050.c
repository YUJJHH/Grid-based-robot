/*
 * mpu6050.c
 *
 *  Created on: May 28, 2024
 *      Author: yuuuj
 */

#include "mpu6050.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;



int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

//float Ax, Ay, Az, Gx, Gy, Gz;

typedef struct{
	float Ax, Ay, Az, Gx, Gy, Gz;
	float Ax_sum;
	float Ay_sum;
	float Az_sum;
	float Ax_avg;
	float Ay_avg;
	float Az_avg;

}typedef_accel_gyro;

typedef_accel_gyro Acceo_Gyro;

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Acceo_Gyro.Ax = Accel_X_RAW/16384.0;
	Acceo_Gyro.Ay = Accel_Y_RAW/16384.0;
	Acceo_Gyro.Az = Accel_Z_RAW/16384.0;

	Acceo_Gyro.Ax_sum += Acceo_Gyro.Ax;
	Acceo_Gyro.Ay_sum += Acceo_Gyro.Ay;
	Acceo_Gyro.Az_sum += Acceo_Gyro.Az;

	static int accel_count=0;

	accel_count++;
		if (accel_count==10) {
			Acceo_Gyro.Ax_avg = Acceo_Gyro.Ax_sum / 10;
			Acceo_Gyro.Ay_avg = Acceo_Gyro.Ay_sum / 10;
			Acceo_Gyro.Az_avg = Acceo_Gyro.Az_sum / 10;

			accel_count = 0;
			Acceo_Gyro.Ax_sum = 0;
			Acceo_Gyro.Ay_sum = 0;
			Acceo_Gyro.Az_sum = 0;
		}
		//printf("%.6f  %.6f  %.6f  %.6f  %.6f %.6f \n\r ",Acceo_Gyro.Ax, Acceo_Gyro.Ay , Acceo_Gyro.Az
		//		, Acceo_Gyro.Ax_avg, Acceo_Gyro.Ay_avg, Acceo_Gyro.Az_avg);
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Acceo_Gyro.Gx = Gyro_X_RAW/131.0;
	Acceo_Gyro.Gy = Gyro_Y_RAW/131.0;
	Acceo_Gyro.Gz = Gyro_Z_RAW/131.0;

	//printf("%.6f  %.6f  %.6f   \r\n",Acceo_Gyro.Gx, Acceo_Gyro.Gy , Acceo_Gyro.Gz);
}












