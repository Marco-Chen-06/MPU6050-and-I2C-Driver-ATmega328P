CC = avr-gcc
CFLAGS += -g -mmcu=atmega328p -DF_CPU=16000000UL -O0

main.elf : main.c utility/twi_master.c utility/timer_16.c mpu6050_driver.c
	${CC} ${CFLAGS} -o $@ $^

.PHONY: clean
clean:
	@rm -rvf *.d *.elf *.o

