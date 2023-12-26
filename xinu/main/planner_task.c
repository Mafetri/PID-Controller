#include <xinu.h>
#include "mat.h"
#include "timer1.h"
#include "data_structures.h"

// Definitions
#define LW_FWD_PORT 4
#define LW_REV_PORT 5
#define RW_FWD_PORT 6
#define RW_REV_PORT 7
#define LW_SPEED_PORT 1
#define LW_SPEED_PORT 2

// Car Configuration
#define WHEEL_DIAMETER 6
#define WHEEL_SEPARATION 19.44

// The min speed for the motors to move, higher -> imprecise, lower -> can't move
#define MIN_SPEED 0.7

// Extern Global Variables
extern volatile car_position car;
extern volatile unsigned char *DDR_D;
extern volatile unsigned char *PORT_D;
extern volatile unsigned char *PIN_D;
extern volatile queue targets;

// Global Variables
double angle_to_target;
double error_angle;
double error_x;
double error_y;
double next_target[2];

// Constants for PID control
#define Kp 1.5
#define Ki 0.01
#define Kd 0.05
#define dT 100

// Move Wheel
// Sets the speed and direction of a wheel
void move_wheel(char side, char direction, double speed)
{
    if (side == 'L')
    {
        if (direction == 'F')
        {
            *(PORT_D) &= ~(1 << LW_REV_PORT);
            *(PORT_D) |= 1 << LW_FWD_PORT;
            car.left_wheel_dir = 'F';
        }
        else if (direction == 'R')
        {
            *(PORT_D) &= ~(1 << LW_FWD_PORT);
            *(PORT_D) |= 1 << LW_REV_PORT;
            car.left_wheel_dir = 'R';
        }
        set_timer1_ocr1a_dutycycle(speed);
    }
    else if (side == 'R')
    {
        if (direction == 'F')
        {
            *(PORT_D) &= ~(1 << RW_REV_PORT);
            *(PORT_D) |= 1 << RW_FWD_PORT;
            car.right_wheel_dir = 'F';
        }
        else if (direction == 'R')
        {
            *(PORT_D) &= ~(1 << RW_FWD_PORT);
            *(PORT_D) |= 1 << RW_REV_PORT;
            car.right_wheel_dir = 'R';
        }
        set_timer1_ocr1b_dutycycle(speed);
    }
}

// Stop Car
// Stops the car
void stop_car() {
    move_wheel('L', 'F', 0);
    move_wheel('R', 'F', 0);
}

// Double Abs
// Returns the double modulus of a double
double double_abs(double x)
{
    if (x < 0)
    {
        return -x;
    }
    else
    {
        return x;
    }
}

// Get Speed
// Returns the final speed from MIN_SPEED to 1 using the PID number
double get_speed(double pid){
    double speed = MIN_SPEED + (pid * WHEEL_SEPARATION / WHEEL_DIAMETER);

    if (speed < MIN_SPEED) {
        speed = MIN_SPEED;
    } else if (speed > 1) {
        speed = 1.0;
    }
    
    return speed;
}

// Compute Error Angle
// Calculates the error angle to the next target
void compute_error_angle() {
    angle_to_target = atan2(error_y, error_x);

    // Error between the actual angle and the angle needed
    error_angle = angle_to_target - car.angle;
    error_angle = atan2(sin(error_angle), cos(error_angle));
}

// Pivot Turn
// Makes the car turns in the same place
double pivot_turn () {
    compute_error_angle();

    if(error_angle < 0) {
        move_wheel('L', 'F', MIN_SPEED);
        move_wheel('R', 'R', MIN_SPEED);
    } else {
        move_wheel('L', 'R', MIN_SPEED);
        move_wheel('R', 'F', MIN_SPEED);
    }

    while(double_abs(error_angle) > 0.1) {
        compute_error_angle();
    }

    stop_car();
}

// Planner
// Using the position, moves the wheels
int planner(void)
{
    // Local Variables
    double proportional;
    double integral = 0.0;
    double derivate = 0.0;
    double pid;
    double left_speed;
    double right_speed;
    queue_dequeue(&targets, &next_target);

    // Sets the Left Wheel pins to output
    *(DDR_D) |= (1 << LW_FWD_PORT) | (1 << LW_REV_PORT);
    // Sets the Right Wheel pins to output
    *(DDR_D) |= (1 << RW_FWD_PORT) | (1 << RW_REV_PORT);

    while(1)
    {
        // Error in x, y and the angle to the target
        error_x = next_target[0] - car.x;
        error_y = next_target[1] - car.y;

        if (double_abs(error_x) < 3 && double_abs(error_y) < 3)
        {
            // If there are more targets, jumps to another iteration
            if(queue_dequeue(&targets, &next_target) == 1) {
                error_x = next_target[0] - car.x;
                error_y = next_target[1] - car.y;
                stop_car();
                sleepms(100);
                pivot_turn();
                continue;
            } else {
                // If not, stops
                stop_car();
                break;
            }
        }

        compute_error_angle();

        // Proportional 
        proportional = error_angle;

        // Integral
        integral = integral + error_angle;

        // Derivative
        derivate = (error_angle - derivate);

        // Gets the total PID
        pid = Kp * proportional + Ki * integral + Kd * derivate;

        // Saves the derivate for the next run
        derivate = error_angle;

        right_speed = get_speed(pid);
        left_speed = get_speed(-pid);

        // Move the wheels according to the PID outputs
        move_wheel('L', 'F', left_speed);
        move_wheel('R', 'F', right_speed);

        sleepms(dT);
    }
}