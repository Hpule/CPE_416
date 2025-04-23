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
 
 #define SENSOR_THRES 160 // threshold for voltage difference between sensors (tells us when we have to turn)
 #define LINE_THRES 185 // threshold for black line 
 
 int16_t prev_error = 0; // this is the previous error value that we will use to calculate the derivative
 int16_t prev_diff = 0;  // this is the previous difference value that we will use to calculate the derivative 

 
 int pid_controller(int16_t error) {
   const float kp = 0.80; // proportional gain
   const float kd = 0.20; // derivative gain
 
  
 
   // calculate the PID command relative to the stop bias (127)
   float command_delta = kp * error + kd * (error - prev_error) + 127;

 
   // clamp the output to the servo's limits (just in case)
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
 
 
 void forward() {
   int left_forward = pid_controller(5); // test 5% so we hopefully get less overshoot 
   int right_forward = pid_controller(-5);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void left() {
   int left_forward = pid_controller(-20); // test 20% so we hopefully get less overshoot
   int right_forward = pid_controller(-100);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void right() {
   int left_forward = pid_controller(100); // test 20% so we hopefully get less overshoot 
   int right_forward = pid_controller(20);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void stop() {
   set_servo(LEFT_SERVO, 127);
   set_servo(RIGHT_SERVO, 127);
 }
 
 void pathing(int16_t left_val, int16_t right_val) {
   int16_t sensor_diff = left_val - right_val;
 
   // accounts for the case where both sensors may be off the line 
   if (left_val < 100 && right_val < 100) { // for debugging purposes
     //use previous turn direction to continue searching for the line
     if (prev_diff < 0) {
       right(); // continue turning right
     } else {
       left(); // continue turning left
     }
     return;
   }
 
   if (abs(sensor_diff) > SENSOR_THRES) {
     if (sensor_diff > 0) {  // left sensor value is higher (turn left)
       left();
       prev_diff = 1;
     } 
     else {
       right(); // right sensor value is higher (turn right)
       prev_diff = -1;
     }
     return;
   }
 
   // sensors are close to each other and above threshold (go forward)
   if (left_val > LINE_THRES && right_val > LINE_THRES) {
     forward();
     prev_diff = 0;
     return;
   }
 
   // just goes forward if no other cases are met
   forward();
 }
 
 int main(void) { 
   init();
   init_lcd();
   init_servo();
 
   while (1) {
     // code to display sensor values (for debugging)
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