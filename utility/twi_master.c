/*
 * twi_master.c -- Interrupt-Driven I2C driver Implementation File
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 */

#include "twi_master.h"
#include "timer_16.h"

#define TIMEOUT_MS 5000

static volatile unsigned char TWI_buffer[TWI_MAX_BUFFER_SIZE];
static volatile unsigned char TWI_msg_size;                  // number of bytes transmitted for message, including address byte
static volatile unsigned char TWI_error_state = TW_NO_STATE; // if something went wrong in the interrupt, this stores the TWSR state
volatile union TWI_status_reg TWI_status_reg = {0};

/*
 * Function scl_parameters
 *
 * Desc     Given an scl_frequency, and twbr & twsr registers, this function analyzes all prescalars (1, 4, 16, 64)
 *          to search for a match to achieve the scl_frequency. If a match is found, twsr and twbr are set to
 *          their approprite values to achieve the scl_frequency. This assumes F_CPU = 16Hz.
 *
 * Input    scl_frequency: Target scl_frequency to set master-transceiver device to.
 *          twbr: Address of AVR TWBR register
 *          twsr: Address of AVR TWSR register
 *
 * Output   Return 1 for scl_frequency successfully found and modified, 0 for failure.
 */
static uint8_t scl_parameters(const uint32_t scl_frequency, volatile uint8_t* const twbr, volatile uint8_t* const twsr) {
  *twsr &= ~((1 << TWPS1) | (1 << TWPS0)); // clear prescalar bits
  uint16_t hypothetical_twbr;
  uint16_t prescalar = 1; // 1, 4, 16, 64
  for (size_t i = 0; i < 4; i++) {
    hypothetical_twbr = ((F_CPU / scl_frequency) - 16) / (2 * prescalar);
    // twbr successfully found
    if (hypothetical_twbr < 256) {
      *twsr += i; // increment TWSR by prescalar bits
      *twbr = hypothetical_twbr;
      return 1;
    }
    prescalar <<= 2; // 1, 4, 16, 64
  }
  // no possible twsr
  return 0;
}

/*
 * Function TWI_wait_transmission_finish
 *
 * Desc     Uses a busy wait until a transmission is finished. If timeout is
 *          reached, update TWI_error_state global variable with TWSR. Handling
 *          the error is up to the caller of the function.
 *
 * Output   Return 1 for timeout, 0 for success
 */
static uint8_t TWI_wait_transmission_finish() {
  unsigned long cur_millis;
  unsigned long prev_millis;

  prev_millis = millis();
  while (TWCR & (1 << TWIE)) {

    cur_millis = millis();

    // update TWI_error_state if timeout
    if (cur_millis - prev_millis >= TIMEOUT_MS) {
      TWI_error_state = TWSR;
      return 1;
    }
  }
  return 0;
}

/*
 * Function TWI_get_failure_state
 *
 * Desc     Gets the most recent TWSR state if it is was an
 *          error state. Intended for debugging use.
 *
 * Output   Return most recent TWSR state since the last error
 */
unsigned char TWI_get_failure_state() {
  // THIS IS A BUSY WAIT (with a timeout)
  TWI_wait_transmission_finish();

  return TWI_error_state;
}

/*
 * Function TWI_init
 *
 * Desc   Sets up TWI master to initial standby state. Also enables global interrupts.
 */
void TWI_init() {
  // enable internal pull-ups for SCL and SDA pins
  DDRC &= ~((1 << PORTC4) | (1 << PORTC5));

  PORTC |= (1 << PORTC4) | (1 << PORTC5);

  // set scl frequency to 100kHz
  scl_parameters(100000, &TWBR, &TWSR);

  // enable twi only, disable everything else
  TWCR |= (1 << TWEN);

  // set data to default
  TWDR = 0xFF;

  // enable global interrupts
  sei();
}

/*
 * Function TWI_start_transceiver_with_data
 *
 * Desc     Initializes twi bus master for transceiving.
 *
 * Input    msg: Pointer to byte array. The first byte must contain the SLA+RW byte.
 *          Consecutive bytes must either contain the data to be sent (write), or
 *          empty locations for data to be stored (read) (for reading, these empty
 *          locations won't actually be accessed/modified in this function. Get the
 *          data using the TWI_receive_data_from_transceiver function).
 *          message_size: Number of bytes to be sent/read, including ADDRESS+RW byte.
 *
 * Output   N/A, but will consider outputting number of bytes read if it helps for debugging
 */
static void TWI_start_transceiver_with_data(unsigned char* msg, unsigned char message_size) {
  // THIS IS A BUSY WAIT (with a timeout)
  TWI_wait_transmission_finish();

  // message size too large, don't process any bytes
  if (TWI_MAX_BUFFER_SIZE < message_size) {
    return; // MUST ERROR HANDLE LATER!!!!!!!!!!!!!!!!!!!!!!!!!!
  }

  TWI_msg_size = message_size;
  TWI_buffer[0] = msg[0];

  if (!(msg[0] & (1 << TWI_RW_BIT))) { // if data is in write mode, copy data into buffer
    for (size_t i = 1; i < message_size; i++) {
      TWI_buffer[i] = msg[i];
    }
  }
  TWI_status_reg.all = 0;

  TWI_error_state = TW_NO_STATE;

  // clear TWINT interrupt flag, generate START, enable TWI, enable TWI interrupts
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

/*
 * Function start_transceiver_with_data
 *
 * Desc     Initializes twi bus master for transceiving, but reuses data already
 *          in TWI_buffer.
 */
static void TWI_start_transceiver() {
  // THIS IS A BUSY WAIT (with a timeout)
  TWI_wait_transmission_finish();

  TWI_status_reg.all = 0;

  TWI_error_state = TW_NO_STATE;

  // clear TWINT interrupt flag, generate START, enable TWI, enable TWI interrupts
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

/*
 * Function receive_data_from_transceiver
 *
 * Desc     Stores whatever data is in the transceiver, into a provided array. This
 *          assumes that a message had already been sent from the slave and stored
 *          in TWI_buffer.
 *
 * Input    msg_ptr: Pointer to byte array. The array is expected to be empty, and able to fit
 *          message_size bytes. This function will populate this array with the data
 *          currently in the transceiver.
 *          message_size: Number of bytes requested, including ADDRESS+RW byte.
 *
 * Output   If there was an error in the previous transmission, return 0.
 *          If no error in previous transmission, return 1.
 */
static unsigned char TWI_receive_data_from_transceiver(unsigned char* msg_ptr, unsigned char message_size) {
  // THIS IS A BUSY WAIT (with a timeout)
  TWI_wait_transmission_finish();

  if (TWI_status_reg.lastTransOK) {
    for (size_t i = 0; i < message_size; i++) {
      msg_ptr[i] = TWI_buffer[i];
    }
  }

  return TWI_status_reg.lastTransOK; // 1 if success, 0 if error
}

/*
 * ISR      TWI_vect (__vect_24 in datasheet)
 *
 * Desc     This ISR handles all TWSR states. It is executed whenever the TWI interrupt is
 *          triggered. TWI_status_reg is updated if any error occurs.
 *
 * Warning  TWCR is set equal to certain bitmasks in this ISR. If you plan to modify
 *          TWCR in any other part of your code, it is highly recommended to change all "TWCR ="
 *          to "TWCR |= or &=" and change the bitmasks so TWCR is conserved.
 */
ISR(TWI_vect) {
  static unsigned char buffer_index;
  switch (TW_STATUS) {
    case TW_START:
    case TW_REP_START:
      buffer_index = 0;
    case TW_MT_SLA_ACK:
    case TW_MT_DATA_ACK:
      if (buffer_index >= TWI_msg_size) {
        TWI_status_reg.lastTransOK = 1;
        // clear TWI interrupt flag, initialize STOP condition, disable TWI interrupt
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (0 << TWIE);
      } else {
        // copy data from current position (should be ADDRESS+RW first) to data register, post increment index
        TWDR = TWI_buffer[buffer_index++];
        // clear TWI interrupt flag, keep TWI enabled, keep TWI interrupt enabled
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
      }
      break;
    case TW_MR_DATA_ACK:
      TWI_buffer[buffer_index++] = TWDR;
    case TW_MR_SLA_ACK:
      if (buffer_index >= TWI_msg_size - 1) {
        // clear TWI interrupt flag, send NACK, keep TWI enabled, keep TWI interrupt enabled
        TWCR = (1 << TWINT) | (0 << TWEA) | (1 << TWEN) | (1 << TWIE);
      } else {
        // clear TWI interrupt flag, send ACK, keep TWI enabled, keep TWI interrupt enabled
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
      }
      break;
    case TW_MR_DATA_NACK:
      TWI_buffer[buffer_index++] = TWDR;
      TWI_status_reg.lastTransOK = 1;
      // clear TWI interrupt flag, initialize STOP condition, keep TWI enabled, disable TWI interrupt
      TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (0 << TWIE);
      break;
    case TW_MR_ARB_LOST: // this condition also encapsulates TW_MT_ARB_LOST
      // clear TWINT interrupt flag, generate START, enable TWI, enable TWI interrupts
      TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
      break;
    case TW_MR_SLA_NACK:
      // initiate repeated start
      TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
      break;
    default: // encapsulates most error states
      TWI_error_state = TWSR;
      TWI_status_reg.lastTransOK = 0;
      // reset TWI interface
      TWCR = (1 << TWEN);
  }
}

/*
 * Function TWI_write_byte
 *
 * Desc     Write a device address, then a register address, then a data byte to the TWI receiver device.
 *
 * Input    dev_address: device address, usually on the datasheet of your I2C receiver device
 *          reg_address: register address you want to write to, also usually on your receiver's datasheet
 *          data_byte: byte of data that you want to write to the device
 */
void TWI_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t data_byte) {
  uint8_t write_buf_size = 3; // dev address, reg address, and data byte to write

  uint8_t TWI_write_buf[write_buf_size];       // contains dev address and reg address and data byte
  TWI_write_buf[0] = ((dev_address << 1) + 0); // R/W bit is 0 for write
  TWI_write_buf[1] = reg_address;
  TWI_write_buf[2] = data_byte;

  TWI_start_transceiver_with_data(TWI_write_buf, write_buf_size);
}

/*
 * Function TWI_write_bytes
 *
 * Desc     Write a device address, then a register address, then multiple data byte to the TWI receiver device.
 *
 * Input    dev_address: device address, usually on the datasheet of your I2C receiver device
 *          reg_address: register address you want to write to, also usually on your receiver's datasheet
 *          data_bytes: pointer/array to the bytes of data that you want to write to the device
 *          num_bytes_to_write: number of data bytes you want to write. Don't count include the ADDRESS+RW byte.
 */
void TWI_write_bytes(uint8_t dev_address, uint8_t reg_address, uint8_t* data_bytes, uint8_t num_bytes_to_write) {
  uint8_t write_buf_size = 2 + num_bytes_to_write; // dev address, reg address, and data bytes to write

  uint8_t TWI_write_buf[write_buf_size];
  TWI_write_buf[0] = ((dev_address << 1) + 0); // R/W bit is 0 for write
  TWI_write_buf[1] = reg_address;

  for (size_t i = 2; i < write_buf_size; i++) {
    TWI_write_buf[i] = 0;
  }
  TWI_start_transceiver_with_data(TWI_write_buf, write_buf_size);
}

/*
 * Function TWI_read_byte
 *
 * Desc     Writes a device address, then a register address. Then, read a data byte from that device.
 *
 * Input    dev_address: device address, usually on the datasheet of your I2C receiver device
 *          reg_address: register address you want to read from, also usually on your receiver's datasheet
 *
 * Output   Returns data byte that was read through I2C. If no data was received, then return 0.
 */
uint8_t TWI_read_byte(uint8_t dev_address, uint8_t reg_address) {
  uint8_t read_buf_size = 2; // dev+W address, reg address, dev+R address, and empty data byte space

  uint8_t TWI_read_buf[read_buf_size];        // store dev and reg address, and also store received data later
  TWI_read_buf[0] = ((dev_address << 1) + 0); // R/W bit is 0 to write the addresses initially
  TWI_read_buf[1] = reg_address;

  TWI_start_transceiver_with_data(TWI_read_buf, read_buf_size);

  TWI_read_buf[0] = ((dev_address << 1) + 1); // R/W bit is now 1 to start reading data (repeated start)

  TWI_start_transceiver_with_data(TWI_read_buf, read_buf_size);

  if (TWI_receive_data_from_transceiver(TWI_read_buf, read_buf_size)) {
    return TWI_read_buf[1];
  }

  // return 0 for failure
  return 0;
}

/*
 * Function TWI_read_bytes
 *
 * Desc     Writes a device address, then a register address. Then, read multiple bytes from that device.
 *
 * Input    dev_address: device address, usually on the datasheet of your I2C receiver device
 *          reg_address: register address you want to read from, also usually on your receiver's datasheet
 *          data_bytes:  pointer/array to the memory location where you want the read data_bytes to be stored
 *          num_bytes_to_read: number of bytes to be read. Don't include the ADDRESS+RW byte.
 *
 * Output   Returns data byte that was read through I2C. If no data was received, then return 0.
 */
void TWI_read_bytes(uint8_t dev_address, uint8_t reg_address, uint8_t* data_bytes, uint8_t num_bytes_to_read) {
  uint8_t read_buf_size = 2 + num_bytes_to_read; // dev+RW address, reg address, and empty data byte spaces -- not all spaces might be used

  uint8_t TWI_read_buf[read_buf_size];        // store dev and reg address, and also store received data later
  TWI_read_buf[0] = ((dev_address << 1) + 0); // R/W bit is 0 to write the addresses initially
  TWI_read_buf[1] = reg_address;
  TWI_start_transceiver_with_data(TWI_read_buf, 2); // write dev_address and read_address

  TWI_read_buf[0] = ((dev_address << 1) + 1); // R/W bit is now 1 to start reading data (repeated start)

  TWI_start_transceiver_with_data(TWI_read_buf, 1 + read_buf_size); // write dev_address and read_address

  if (TWI_receive_data_from_transceiver(TWI_read_buf, 1 + num_bytes_to_read)) { // will read dev_address and the number of bytes to read
    for (size_t i = 0; i < num_bytes_to_read; i++) {
      data_bytes[i] = TWI_read_buf[i + 1]; // skip 1 index because index 0 is address+rw
    }
  }
}