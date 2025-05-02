/**
 * Name: Hector Pule and Wilson Yu
 * Lab 3 part 1
 * Description: Proportional controller 
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>

 typedef struct motor_command { // struct for the motor command (left and right speed)
     uint8_t left_speed;
     uint8_t right_speed;
 } motor_command;
 
 motor_command compute_proportional(uint8_t left, uint8_t right); // function prototype for PID controller (only proportional) 

 motor_command compute_proportional(uint8_t left, uint8_t right) {
    motor_command both_commands; // initialize a motor_command struct
    float kp = 0.5; // proportional gain
    float left_command = kp * (left) + 127; // equation with only proportional 
    float right_command = kp * (right) + 127; // equation with only proportional
    both_commands.left_speed = left_command; // assign the left command to the struct
    both_commands.right_speed = right_command; // assign the right command to the struct

    return both_commands; // return the left and right command
 }

 int main(void) {
    uint8_t left_sensor = analog(3); // get the left and right sensor values
    uint8_t right_sensor = analog(4);

    // use the two sensor values to get motor commands
    motor_command mc = compute_proportional(left_sensor, right_sensor);
    set_servo(2, mc.left_speed); // set the left motor speed
    set_servo(3, mc.right_speed); // set the right motor speed
 }