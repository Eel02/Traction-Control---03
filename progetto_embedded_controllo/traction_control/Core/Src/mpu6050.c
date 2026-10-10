/*
 * mpu6050.c
 *
 *  Created on: Jul 9, 2026
 *      Author: soria
 */
#include "mpu6050.h"

/* Initialize the IMU */
HAL_StatusTypeDef imu_init(mpu6050_t *imu, I2C_HandleTypeDef *hi2c, uint8_t address)
{
	uint8_t whoami;
	imu->hi2c = hi2c;
	imu->address = address;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->hi2c, imu->address, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100);
	if (status == HAL_OK) {
		  if (whoami == WHO_AM_I_VAL){
			  uint8_t data;

			  /* power management register 0X6B must be all 0s to wake the sensor up */
			  data = 0;
			  if ( HAL_I2C_Mem_Write(imu->hi2c, imu->address, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
				  return HAL_ERROR;
			  }

			  /* Set DATA RATE of 1KHz by writing SMPLRT_DIV register */
			  data = 0x07;
			  if ( HAL_I2C_Mem_Write(imu->hi2c, imu->address, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
				  return HAL_ERROR;
			  }

			  /*  Set accelerometer configuration in ACCEL_CONFIG Register */
			  /* XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->  2g */
			  data = 0x00;
			  if ( HAL_I2C_Mem_Write(imu->hi2c, imu->address, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
				  return HAL_ERROR;
			  }

			  /* Set Gyroscope configuration in GYRO_CONFIG Register */
			  /* XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->  250 deg/s */
			  data = 0x00;
			  if ( HAL_I2C_Mem_Write(imu->hi2c, imu->address, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
				  return HAL_ERROR;
			  }

		  } else {
			  return HAL_ERROR;
		  }
	} else {
		return HAL_ERROR;
	}

	return HAL_OK;
}
//Funzione per calibrazione IMU
void imu_calibrate(mpu6050_t *imu) {
    int32_t sum_accel_x = 0;
    int32_t sum_accel_z = 0;
    int32_t sum_gyro_y = 0;
    int32_t sum_gyro_z = 0;
    uint8_t temp_buffer[14];

    //Media di 100 letture a veicolo fermo in piano
    for(int i = 0; i < 100; i++) {
        if(HAL_I2C_Mem_Read(imu->hi2c, imu->address, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, temp_buffer, 14, 100) == HAL_OK) {
            sum_accel_x += (int16_t)(temp_buffer[0] << 8 | temp_buffer[1]);
            sum_accel_z += (int16_t)(temp_buffer[4] << 8 | temp_buffer[5]);
            sum_gyro_y  += (int16_t)(temp_buffer[10] << 8 | temp_buffer[11]);
            sum_gyro_z  += (int16_t)(temp_buffer[12] << 8 | temp_buffer[13]);
        }
        HAL_Delay(5);
    }
    imu->imu_bias.accel_x = (int16_t)(sum_accel_x / 100);
    imu->imu_bias.accel_z = (int16_t)((sum_accel_z / 100) - 16384); //Sottrae 1g nominale
    imu->imu_bias.gyro_y  = (int16_t)(sum_gyro_y / 100);
    imu->imu_bias.gyro_z  = (int16_t)(sum_gyro_z / 100);
}
