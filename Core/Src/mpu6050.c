/*
 * mpu6050.c
 *
 *  Created on: Feb 7, 2021
 *      Author: Pawel
 */

#include "mpu6050.h"

uint8_t init_mpu(MPU6050 *data, I2C_HandleTypeDef *i2c, uint32_t acc_sens, uint32_t gyro_sens)
{
    uint8_t check;
    uint8_t Data;

    data ->i2c = i2c;
    data ->acc_sens = acc_sens;
    data ->gyro_sens = gyro_sens;

    // check device ID WHO_AM_I
    HAL_I2C_Mem_Read(data ->i2c, MPU6050_ADDRESS, MPU6050_WHO_AM_I, 1, &check, 1, I2C_TIMEUOT);

    if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1, &Data, 1, I2C_TIMEUOT);

        // Set DATA RATE of 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 1, &Data, 1, I2C_TIMEUOT);

        // Set accelerometer configuration in ACCEL_CONFIG Register -> 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEUOT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register -> 250 o/s
        Data = 0x00;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &Data, 1, I2C_TIMEUOT);


        return 1;
    }
    return 0;
}

void get_acc_data(MPU6050 *data)
{
	uint8_t temp_data[6] = {0};
	HAL_I2C_Mem_Read(data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 1, temp_data, 6, I2C_TIMEUOT);

	//scale data in [g]
	data->acc_data[0] = (int16_t) (temp_data[0] << 8 | temp_data[1]) /16384.0;;
	data->acc_data[1] = (int16_t) (temp_data[2] << 8 | temp_data[3]) /16384.0;;
	data->acc_data[2] = (int16_t) (temp_data[4] << 8 | temp_data[5]) /16384.0;;

}

void get_gyro_data(MPU6050 *data)
{
	uint8_t temp_data[6] = {0};
	HAL_I2C_Mem_Read(data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 1, temp_data, 6, I2C_TIMEUOT);

	//scale data in [°/s]
	data->gyro_data[0] = (int16_t) (temp_data[0] << 8 | temp_data[1]) /131.0;
	data->gyro_data[1] = (int16_t) (temp_data[2] << 8 | temp_data[3]) /131.0;
	data->gyro_data[2] = (int16_t) (temp_data[4] << 8 | temp_data[5]) /131.0;
}

void save_act_position(MPU6050 *data)
{
	//save act pos
	data->save_pos[0] = data->axis[0];
	data->save_pos[1] = data->axis[1];
	data->save_pos[2] = data->axis[2];
}

void check_position(MPU6050 *data)
{
	//accelerometer angles
	float roll_acc = atan2(data->acc_data[0], sqrt(sqrt(data->acc_data[1]) + sqrt(data->acc_data[2])))/(M_PI/180);
	float pitch_acc = atan2(data->acc_data[1], sqrt(sqrt(data->acc_data[0]) + sqrt(data->acc_data[2])))/(M_PI/180);
	float yaw_acc = atan2(data->acc_data[2], sqrt(sqrt(data->acc_data[0]) + sqrt(data->acc_data[1])))/(M_PI/180);

	//complementary filter - Final estimated angle
	data->axis[0] = 0.98*data->gyro_after_integration[0]+(1.0-0.98)*roll_acc; //roll
	data->axis[1] = 0.98*data->gyro_after_integration[1]+(1.0-0.98)*pitch_acc; //pitch
	data->axis[2] = 0.98*data->gyro_after_integration[2]+(1.0-0.98)*yaw_acc; //yaw
}
