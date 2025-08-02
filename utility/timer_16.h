/*
 * timer_16.h -- Interrupt-Driven AVR 16-bit Timer Driver
 * Copyright (C) 2025  Marco Chen <marcochen.work@gmail.com>
 */

#ifndef TIMER_16_H
#define TIMER_16_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <compat/twi.h>
#include <stddef.h>
#include <stdint.h>

uint16_t read_tcnt1();
void write_tcnt1(uint16_t mask);
void timer_init();
unsigned long millis();

#endif