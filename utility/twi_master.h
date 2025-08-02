/*
 * twi_master.c -- Interrupt-Driven I2C driver Header File
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 */

#ifndef TWI_MASTER_H
#define TWI_MASTER_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <compat/twi.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define TW_NO_STATE 0 // represents no state
#define TWI_RW_BIT 0 // bit position of read/write bit in address
#define TWI_MAX_BUFFER_SIZE 32 // set this as your largest desired TWI_buffer size

// NOT related to TWSR, this is just a made-up register for debugging success or failed communications
union TWI_status_reg {
  unsigned char all;
  struct {
    unsigned char lastTransOK : 1; // 1 for success, 0 for error
    unsigned char unusedBits : 7;
  };
};

// All TWI Implementation functions
void TWI_init();
unsigned char TWI_get_failure_state();
void TWI_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t data_byte);
void TWI_write_bytes(uint8_t dev_address, uint8_t reg_address, uint8_t* data_bytes, uint8_t num_bytes_to_write);
uint8_t TWI_read_byte(uint8_t dev_address, uint8_t reg_address);
void TWI_read_bytes(uint8_t dev_address, uint8_t reg_address, uint8_t* data_bytes, uint8_t num_bytes_to_read);

#endif
