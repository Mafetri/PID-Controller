#include <xinu.h>
#include <avr/interrupt.h>
#include "serial.h"
#include "mat.h"
#include "timer1.h"
#include "data_structures.h"

// Process with external code
extern int serial_output(void);
extern int rtc(void);
extern void encoder_init(void);
extern int locator(void);
extern int planner(void);

// Global Variables
unsigned long rtc_miliseconds;
volatile queue targets;
volatile car_position car;

// Ports
volatile unsigned char *DDR_B = 0x24;
volatile unsigned char *PORT_B = 0x25;
volatile unsigned char *PIN_B = 0x23;
volatile unsigned char *DDR_D = 0x2A;
volatile unsigned char *PORT_D = 0x2B;
volatile unsigned char *PIN_D = 0x29;

// Main process
int main(void)
{
	serial_init();
	init_timer1();
	encoder_init();
	queue_init(&targets);
	queue_enqueue_values(&targets, 60.0, 0.0);
	queue_enqueue_values(&targets, 60.0, 120.0);
	queue_enqueue_values(&targets, 0.0, 120.0);
	queue_enqueue_values(&targets, 0.0, 0.0);

	// Create and run process
	resume(create(locator, 192, 19, "loc", 0));
	resume(create(planner, 192, 19, "pla", 0));
	resume(create(serial_output, 128, 19, "s_o", 0));

	*DDR_B |= 1 << 5;
	while (1)
	{
		sleepms(200);
		*PORT_B |= 1 << 5; // high
		sleepms(200);
		*PORT_B ^= 1 << 5; // low
	}

	return 0;
}
