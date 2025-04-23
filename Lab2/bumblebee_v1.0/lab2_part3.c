/**
 * Name: Hector Pule and Wilson Yu
 * Lab 2 part 3
 * Description: line following Harabot using IR sensors with PID controller
 * 
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 
 #define LEFT_SENSOR 3
 #define RIGHT_SENSOR 4 
 
 #define LEFT_SERVO 2
 #define RIGHT_SERVO 3
 #define SERVO_MAX 157 
 #define SERVO_MIN 97
 
 #define SENSOR_THRES 60 // threshold for voltage difference between sensors (tells us when we have to turn)
 #define LINE_THRES 185 // threshold for black line 
 
 int16_t prev_error = 0; // this is the previous error value that we will use to calculate the derivative
 int16_t integral = 0;   // this is the accumulated error value for the integral component
 int16_t prev_diff = 0;  // this is the previous difference value that we will use to calculate the derivative 
 
 // Maximum value for the integral term to prevent windup
 #define INTEGRAL_MAX 500  
 #define INTEGRAL_MIN -500
 
 int pid_controller(int16_t error) {
   // Controller gains - TUNE THESE VALUES!
   const float kp = 0.70; // Proportional gain
   const float ki = 0.0085; // Integral gain - start small and adjust
   const float kd = 0.30; // Derivative gain
 
   // Update integral term with current error
   integral += error;
   
   // Anti-windup - limit the integral term to prevent excessive accumulation
   if (integral > INTEGRAL_MAX) {
     integral = INTEGRAL_MAX;
   } else if (integral < INTEGRAL_MIN) {
     integral = INTEGRAL_MIN;
   }
 
   // Calculate the PID command relative to the stop bias
   float command_delta = kp * error + ki * integral + kd * (error - prev_error) + 127;
 
   // Clamp the output to the servo's limits
   int final_command;
   if (command_delta > SERVO_MAX) { // if the raw is over the max, set it to the max
       final_command = SERVO_MAX;
   } 
   else if (command_delta < SERVO_MIN) { // if the raw is under the min, set it to the min
       final_command = SERVO_MIN;     
   } 
   else { 
       final_command = (int)round(command_delta); // round instead of truncating
   }
 
   prev_error = error; // update the previous error for the next iteration
   return final_command; // return the final command value
 }
 
 // Function to reset the integral term when necessary
 void reset_pid_integral() {
   integral = 0;
 }
 
 void forward() {
   int left_forward = pid_controller(20); // test 75% so we hopefully get less overshoot 
   int right_forward = pid_controller(-20);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void left() {
   int left_forward = pid_controller(-23); // test 75% so we hopefully get less overshoot
   int right_forward = pid_controller(-100);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void right() {
   int left_forward = pid_controller(100); // test 75% so we hopefully get less overshoot 
   int right_forward = pid_controller(23);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void stop() {
   set_servo(LEFT_SERVO, 127);
   set_servo(RIGHT_SERVO, 127);
   // Reset the integral term when stopping to prevent accumulated error
   reset_pid_integral();
 }
 
 void pathing(int16_t left_val, int16_t right_val) {
   int16_t sensor_diff = left_val - right_val;
 
   // Extreme case: Both sensors detect very low values (possibly off the line)
   if (left_val < 100 && right_val < 100) {
     // Use previous turn direction to continue searching for the line
     if (prev_diff < 0) {
       right(); // Continue turning right
       led_on(1);  // Turn on LED1 for right turn
       led_off(0);
     } else {
       left(); // Continue turning left
       led_on(0);  // Turn on LED0 for left turn
       led_off(1);
     }
     return;
   }
 
   // Case 1: Significant difference between sensors - need to turn
   if (abs(sensor_diff) > SENSOR_THRES) {
     if (sensor_diff > 0) {
       // Left sensor sees more black - turn left
       left();
       prev_diff = 1;
       led_on(0);  // Turn on LED0 for left turn
       led_off(1);
     } else {
       // Right sensor sees more black - turn right
       right();
       prev_diff = -1;
       led_on(1);  // Turn on LED1 for right turn
       led_off(0);
     }
     return;
   }
 
   // Case 2: Sensors are close to each other and above threshold - go forward
   if (left_val > LINE_THRES && right_val > LINE_THRES) {
     forward();
     prev_diff = 0;
     // Turn off both LEDs when going straight
     led_off(0);
     led_off(1);
     return;
   }
 
   // Default case: If nothing else matches, try to go forward
   forward();
   // Turn off both LEDs when going straight
   led_off(0);
   led_off(1);
 }
 
 int main(void) { 
   init();
   init_lcd();
   init_servo();
 
   // Initialize the integral term
   integral = 0;
 
   while (1) {
     // code to display sensor values 
     uint8_t left = analog(LEFT_SENSOR);
     uint8_t right = analog(RIGHT_SENSOR);
     char left_msg[17];
     char right_msg[17];
    
     lcd_cursor(0, 0);
     sprintf(left_msg, "L:%3d", left);
     print_string(left_msg);
    
     lcd_cursor(0, 1);
     sprintf(right_msg, "R:%3d", right);
     print_string(right_msg);
 
     pathing(left, right);
   }
 }