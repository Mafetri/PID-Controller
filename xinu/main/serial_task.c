#include <xinu.h>
#include "serial.h"
#include "math.h"
#include "data_structures.h"

// Global Variables
extern unsigned int left_wheel_ticks;
extern unsigned int right_wheel_ticks;
extern volatile car_position car;

// Another Process
int serial_output(void)
{
	while (1)
	{
		serial_put_str("(");
		serial_put_double(car.x, 3);
		serial_put_str(", ");
		serial_put_double(car.y, 3);
		serial_put_str(") Angle: ");
		serial_put_double(car.angle, 3);
		serial_put_new_line();

		sleepms(500);
	}
}
