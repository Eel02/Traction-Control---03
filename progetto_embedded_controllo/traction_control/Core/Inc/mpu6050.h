/*
 * mpu6050.h
 *
 *  Created on: Jul 9, 2026
 *      Author: soria
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"


/* MPU6050 registers' addresses */
#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_1_REG 		0x6B
#define SMPLRT_DIV_REG 		0x19
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_CONFIG_REG 	0x1B
#define GYRO_XOUT_H_REG 	0x43

/* IMU Address */
#define IMU_ADDR			0xD0

/* Default value of the WHO_AM_I register */
#define WHO_AM_I_VAL		0x68

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} mpu6050_data_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	mpu6050_data_t imu_bias;
	uint8_t address;
} mpu6050_t;


HAL_StatusTypeDef imu_init(mpu6050_t *imu, I2C_HandleTypeDef *hi2c, uint8_t address);

void imu_calibrate(mpu6050_t *imu);


#endif
