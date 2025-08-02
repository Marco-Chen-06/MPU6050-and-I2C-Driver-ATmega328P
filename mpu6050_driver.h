#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "utility/twi_master.h"
#include <stdint.h>

#define MPU6050_ADDRESS 0x68 
#define MPU6050_CONFIG 0x1A 
#define MPU6050_GYRO_CONFIG 0x1B 
#define MPU6050_ACC_CONFIG 0x1C 
#define MPU6050_INT_ENABLE 0x38 
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_PWR_MGMT_1 0x6B 
#define MPU6050_WHO_AM_I 0x75

uint8_t MPU_init();
void MPU_get_ang_vel(float * data_buf);
void MPU_get_acc(float * data_buf);
#endif