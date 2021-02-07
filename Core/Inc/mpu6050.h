/*
 * mpu6050.h
 *
 *  Created on: Feb 7, 2021
 *      Author: Pawel
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdbool.h>
#include "Math.h"
#include "stm32l4xx_hal.h"

#define MPU6050_ADDRESS 			0xD0
#define MPU6050_XG_OFFS_TC 			0x00
#define MPU6050_YG_OFFS_TC 			0x01
#define MPU6050_ZG_OFFS_TC 			0x02
#define MPU6050_X_FINE_GAIN 		0x03
#define MPU6050_Y_FINE_GAIN 		0x04
#define MPU6050_Z_FINE_GAIN 		0x05
#define MPU6050_XA_OFFS_H 			0x06
#define MPU6050_XA_OFFS_L_TC 		0x07
#define MPU6050_YA_OFFS_H 			0x08
#define MPU6050_YA_OFFS_L_TC 		0x09
#define MPU6050_ZA_OFFS_H 			0x0A
#define MPU6050_ZA_OFFS_L_TC 		0x0B
#define MPU6050_XG_OFFS_USRH 		0x13
#define MPU6050_XG_OFFS_USRL 		0x14
#define MPU6050_YG_OFFS_USRH 		0x15
#define MPU6050_YG_OFFS_USRL 		0x16
#define MPU6050_ZG_OFFS_USRH 		0x17
#define MPU6050_ZG_OFFS_USRL 		0x18
#define MPU6050_SMPLRT_DIV 			0x19
#define MPU6050_CONFIG 				0x1A
#define MPU6050_GYRO_CONFIG 		0x1B
#define MPU6050_ACCEL_CONFIG 		0x1C
#define MPU6050_FF_THR 				0x1D
#define MPU6050_FF_DUR 				0x1E
#define MPU6050_MOT_THR 			0x1F
#define MPU6050_MOT_DUR 			0x20
#define MPU6050_ZRMOT_THR 			0x21
#define MPU6050_ZRMOT_DUR 			0x22
#define MPU6050_FIFO_EN 			0x23
#define MPU6050_I2C_MST_CTRL 		0x24
#define MPU6050_I2C_SLV0_ADDR 		0x25
#define MPU6050_I2C_SLV0_REG 		0x26
#define MPU6050_I2C_SLV0_CTRL 		0x27
#define MPU6050_I2C_SLV1_ADDR 		0x28
#define MPU6050_I2C_SLV1_REG 		0x29
#define MPU6050_I2C_SLV1_CTRL 		0x2A
#define MPU6050_I2C_SLV2_ADDR 		0x2B
#define MPU6050_I2C_SLV2_REG 		0x2C
#define MPU6050_I2C_SLV2_CTRL 		0x2D
#define MPU6050_I2C_SLV3_ADDR 		0x2E
#define MPU6050_I2C_SLV3_REG 		0x2F
#define MPU6050_I2C_SLV3_CTRL 		0x30
#define MPU6050_I2C_SLV4_ADDR 		0x31
#define MPU6050_I2C_SLV4_REG 		0x32
#define MPU6050_I2C_SLV4_DO 		0x33
#define MPU6050_I2C_SLV4_CTRL 		0x34
#define MPU6050_I2C_SLV4_DI 		0x35
#define MPU6050_I2C_MST_STATUS 		0x36
#define MPU6050_INT_PIN_CFG 		0x37
#define MPU6050_INT_ENABLE 			0x38
#define MPU6050_DMP_INT_STATUS 		0x39
#define MPU6050_INT_STATUS 			0x3A
#define MPU6050_ACCEL_XOUT_H 		0x3B
#define MPU6050_ACCEL_XOUT_L 		0x3C
#define MPU6050_ACCEL_YOUT_H 		0x3D
#define MPU6050_ACCEL_YOUT_L 		0x3E
#define MPU6050_ACCEL_ZOUT_H 		0x3F
#define MPU6050_ACCEL_ZOUT_L 		0x40
#define MPU6050_TEMP_OUT_H 			0x41
#define MPU6050_TEMP_OUT_L 			0x42
#define MPU6050_GYRO_XOUT_H 		0x43
#define MPU6050_GYRO_XOUT_L 		0x44
#define MPU6050_GYRO_YOUT_H 		0x45
#define MPU6050_GYRO_YOUT_L 		0x46
#define MPU6050_GYRO_ZOUT_H 		0x47
#define MPU6050_GYRO_ZOUT_L 		0x48
#define MPU6050_MOT_DETECT_STATUS 	0x61
#define MPU6050_I2C_SLV0_DO 		0x63
#define MPU6050_I2C_SLV1_DO 		0x64
#define MPU6050_I2C_SLV2_DO 		0x65
#define MPU6050_I2C_SLV3_DO 		0x66
#define MPU6050_ACC_SENS_2G			0x01
#define MPU6050_ACC_SENS_4G			0x02
#define MPU6050_ACC_SENS_8G			0x03
#define MPU6050_ACC_SENS_16G		0x04
#define MPU6050_GYR_SENS_250		0x01
#define MPU6050_GYR_SENS_500		0x02
#define MPU6050_GYR_SENS_1000		0x03
#define MPU6050_GYR_SENS_2000		0x04
#define MPU6050_I2C_MST_DELAY_CTRL 	0x67
#define MPU6050_SIGNAL_PATH_RESET 	0x68
#define MPU6050_MOT_DETECT_CTRL 	0x69
#define MPU6050_USER_CTRL 			0x6A
#define MPU6050_PWR_MGMT_1 			0x6B
#define MPU6050_PWR_MGMT_2 			0x6C
#define MPU6050_BANK_SEL 			0x6D
#define MPU6050_MEM_START_ADDR 		0x6E
#define MPU6050_MEM_R_W 			0x6F
#define MPU6050_DMP_CFG_1 			0x70
#define MPU6050_DMP_CFG_2 			0x71
#define MPU6050_FIFO_COUNTH 		0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W 			0x74
#define MPU6050_WHO_AM_I 			0x75

#define I2C_TIMEUOT					100 //timeout i2c in [ms]

typedef struct {

	//init parameter
	//using i2c
	I2C_HandleTypeDef *i2c;
	//Sensitivity Scale Factor
	uint32_t gyro_sens;
	uint32_t acc_sens;

	//parameters changed by buttons
	//free degree system
	uint8_t free_degree;
	//reference position
	float save_pos[3];

	//live position data
	//acc in [g]
	uint16_t acc_data[3];
	//gyro in [º/s]
	uint16_t gyro_data[3];
	//gyro after integration in [º]
	float gyro_after_integration[3];
	//final axis in space
	float axis[3];
} MPU6050;


uint8_t init_mpu(MPU6050 *data, I2C_HandleTypeDef *i2c, uint32_t acc_sens, uint32_t gyro_sens);
void get_acc_data(MPU6050 *data);
void get_gyro_data(MPU6050 *data);
void save_act_position(MPU6050 *data);



#endif /* INC_MPU6050_H_ */
