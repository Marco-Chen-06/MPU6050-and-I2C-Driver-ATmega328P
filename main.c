/*
 * main.c -- Example use of MPU6050 library. Fetch gyro values and print them.
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 */

#include "utility/timer_16.h"
#include "utility/twi_master.h"
#include "mpu6050_driver.h"
#include <stdbool.h>
#include <stdio.h>

int main() {
  float data_buf[3];

  timer_init();

  if (!(MPU_init())) {
    // MPU initialization failed, implement error handling here
    return 1;
  }  

  for (;;) {
    // populate data_buf with gyroscope readings using I2C
    MPU_get_ang_vel(data_buf);

    // prints gyro values (angular velocity) for x, y, and z directions with MPU6050
    printf("gyro x: %f    gyro y:%f    gyro z:%f\n", data_buf[0], data_buf[1], data_buf[2]);
  }

  return 0;
}

// if you are using gdb, this dprintf statement might be useful:
// dprintf 24, "gyro x: %f    gyro y:%f    gyro z:%f\n", data_buf[0], data_buf[1], data_buf[2]

