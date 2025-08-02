/*
 * twi_master.h -- Interrupt-Driven I2C driver Implementation File
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 */

#include "mpu6050_driver.h"

/*
 * Function MPU_init
 *
 * Desc     Initializes TWI, specifically for MPU6050. Handles all default gyroscope configurations.
 *          Also reads from MPU6050_WHO_AM_I to test if communication is successful.
 *
 * Output   Return 1 if success (MPU6050_WHO_AM_I was received). Return 0 if failure.
 */
uint8_t MPU_init() {
    TWI_init();
    TWI_write_byte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
    TWI_write_byte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x01);
    TWI_write_byte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, (1 << 4));
    TWI_write_byte(MPU6050_ADDRESS, MPU6050_ACC_CONFIG, 0x00); // default, 250 degrees per second

    TWI_write_byte(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 0x00);
    TWI_write_byte(MPU6050_ADDRESS, MPU6050_SIGNAL_PATH_RESET, 0x00);

    if (TWI_read_byte(MPU6050_ADDRESS, MPU6050_WHO_AM_I) != MPU6050_ADDRESS) {
        return 0; // failed, WHOAMI is not the address
    } 

    return 1;
}

/*
 * Function MPU_get_ang_vel
 *
 * Desc     Reads from MPU6050 gyroscope. Will populate a buffer with all received gyro (angular velocity) values
 *          as floats from the MPU6050.
 *
 * Input    data_buf: pointer to address where received gyro values should be stored. Be sure that if this is an 
 *          array, it should be at least 3 elements long. Each x, y, and z gyro value is stored as a float.
 */
void MPU_get_ang_vel(float * data_buf) {
    uint8_t raw_buf[6];
    TWI_read_bytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, raw_buf, 6);

    // gyro default config is 250 deg/sec, need to divide by 65.5
    data_buf[0] = ((float)((((int16_t)raw_buf[0]) << 8) | raw_buf[1]))/65.5;

    data_buf[1] = ((float)((((int16_t)raw_buf[2]) << 8) | raw_buf[3]))/65.5;

    data_buf[2] = ((float)((((int16_t)raw_buf[4]) << 8) | raw_buf[5]))/65.5;
}

/*
 * Function MPU_get_acc
 *
 * Desc     Reads from MPU6050 accelerometer. Will populate a buffer with all received acceleration values from MPU6050.
 *
 * Input    data_buf: pointer to address where received accelerometer values should be stored. Be sure that if this is an 
 *          array, it should be at least 3 elements long. Each x, y, and z gyro value is stored as a float.
 */

void MPU_get_acc(float * data_buf) {
    uint8_t raw_buf[6];
    TWI_read_bytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, raw_buf, 6);

    // accelerometer default config is +-2g, need to divide by 16384 
    data_buf[0] = ((float)((((int16_t)raw_buf[0]) << 8) | raw_buf[1]))/16384;

    data_buf[1] = ((float)((((int16_t)raw_buf[2]) << 8) | raw_buf[3]))/16384;

    data_buf[2] = ((float)((((int16_t)raw_buf[4]) << 8) | raw_buf[5]))/16384;
}


