# MPU6050_and_I2C_Driver_ATmega328P
An Interrupt-Driven I2C driver was written using AVR C, for the ATmega328P. This I2C driver was used to make an MPU6050 driver, to read accelerometer and gyroscope data from the MPU6050 registers. An Interrupt-Driven timer library was also implemented using the AVR microcontroller's 16-bit timer. 

# main.c
cain.c documents an example reading of the gyroscope data (angular velocity)

#twi_master.c 
twi_master.c manages the AVR TWI port registers to allow for the ATmega328P to be a Master-Transceiver device in I2C

#mpu6050_driver.c
mpu6050_driver.c uses twi_master.c to get MPU6050 data readings

#timer_16.c
timer_16.c is an interrupt-drive 16-bit timer

This code should be compatible with other AVR microcontrollers. 

# Running the Code
- Clone the Repository
- Make appropriate wiring connections
- Run the Makefile or compile with your preferred method.

