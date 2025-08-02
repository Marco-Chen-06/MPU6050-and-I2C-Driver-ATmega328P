/*
 * timer_16.c -- Interrupt-Driven AVR 16-bit Timer Driver
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 * 
 * Disclaimer: This timer is limited to only updating its millis count every 4 milliseconds, so
 * it is not recommended to use this library if you require precise timed events.
 */

#include "timer_16.h"

volatile unsigned long timer1_millis = 0;
volatile unsigned long timer1_overflow_count = 0;

#define OVERFLOW_MILLIS 4 // number of millis per timer1 overflow

/*
 * Function read_tcnt1
 *
 * Desc     Read from the 16-bit TCNT1 register. Since this is an atomic operation, disable
 *          global interrupts during this reading.
 *
 * Output   16-bit value representing the state of the TCNT1 register
 */

// accessing 16 bit registers are atomic operations, disable interrupts
uint16_t read_tcnt1() {
  // critical section, disable interrupts
  uint8_t temp_sreg = SREG;
  cli();

  uint16_t mask = TCNT1;

  SREG = temp_sreg;

  return mask;
}

/*
 * Function write_tcnt1
 *
 * Desc     Modify the 16-bit TCNT1 register. Since this is an atomic operation, disable
 *          global interrupts during this write.
 */
void write_tcnt1(uint16_t mask) {
  // critical section, disable interrupts
  uint8_t temp_sreg = SREG;
  cli();

  TCNT1 = mask;

  SREG = temp_sreg;
}

/*
 * Function timer_init
 *
 * Desc     Initialze the 16-bit timer to normal mode with a prescalar of 1.
 */
void timer_init() {
  // initialize normal mode
  TCCR1A &= ~((1 << WGM11) & (1 << WGM10));
  TCCR1B &= ~((1 << WGM13) & (1 << WGM12));

  // no prescaling
  TCCR1B |= (1 << CS10);

  // enable timer overflow interrupt
  TIMSK1 |= (1 << TOIE1);
}

/*
 * ISR      TIMER1_OVF_vect (__vect_13 in datasheet)
 *
 * Desc     Handle the updating of millis on every timer1 overflow.
 */
ISR(TIMER1_OVF_vect) {
  unsigned long millis = timer1_millis;

  millis += OVERFLOW_MILLIS;

  timer1_millis = millis;
  timer1_overflow_count++;
}

/*
 * Function millis 
 *
 * Desc     Keeps track of milliseconds since program began running on Atmega328P
 * 
 * Output   Number of milliseconds since program began running
 * 
 */
unsigned long millis() {
  unsigned long millis;

  // critical section, disable interrupts
  uint8_t temp_sreg = SREG;
  cli();

  millis = timer1_millis;

  SREG = temp_sreg;

  return millis;
}