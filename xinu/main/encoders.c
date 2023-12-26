#include <xinu.h>
#include <avr/interrupt.h>
#include "data_structures.h"

// Global Variables
extern volatile car_position car;

#define INT0 0
#define INT1 1
#define ISC00 0
#define ISC01 1
#define ISC10 2
#define ISC11 3

// Enalbes the interrupts of the D2 and D3 ports on rising edge
void encoder_init() {
    EIMSK |= (1 << INT0) | (1 << INT1);
    EICRA |= (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
    sei();
}

ISR(INT0_vect) {
    if(car.right_wheel_dir == 'R'){
        car.right_ticks--;
    } else {
        car.right_ticks++;
    }
}

ISR(INT1_vect) {
    if(car.left_wheel_dir == 'R'){
        car.left_ticks--;
    } else {
        car.left_ticks++;
    }
}

