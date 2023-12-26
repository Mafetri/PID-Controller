#include <xinu.h>
#include <mat.h>
#include "data_structures.h"

#define WHEEL_DIAMETER 6
#define WHEEL_SEPARATION 19.44
#define ENCODER_RES 20
#define PI 3.14159265359

// Global Variables
extern volatile car_position car;

// Locator
// Using the ticks of each wheel, calculates the position of the robot
int locator(void)
{
	// Local Variables
	int delta_left_ticks = 0;
	int delta_right_ticks = 0;
	int last_left_ticks = 0;
	int last_right_ticks = 0;
	float left_distance = 0.0;
	float right_distance = 0.0;
	float center_distance = 0.0;

	while (1)
	{
		// Read the wheel tick counts from last measurement
		delta_left_ticks = car.left_ticks - last_left_ticks;
		delta_right_ticks = car.right_ticks - last_right_ticks;
		last_left_ticks = car.left_ticks;
		last_right_ticks = car.right_ticks;

		// Calculate wheel odometry
		left_distance = (PI * WHEEL_DIAMETER / ENCODER_RES) * delta_left_ticks;
		right_distance = (PI * WHEEL_DIAMETER / ENCODER_RES) * delta_right_ticks;
		center_distance = (left_distance + right_distance) / 2.0;

		// Update robot position and angle
		car.x += center_distance * cos(car.angle);
		car.y += center_distance * sin(car.angle);
		car.angle += (right_distance - left_distance) / WHEEL_SEPARATION;

		sleepms(100);
	}
}