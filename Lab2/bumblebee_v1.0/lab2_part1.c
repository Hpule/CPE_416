/**
 * Name: Hector Pule and Wilson Yu
 * Lab 2 part 1
 * Description: Control servos 2 and 3 with left motor power boost
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 
 // Motor power adjustment (positive to boost left motor)
 #define LEFT_BOOST 20  // Boost left motor power
 
 // Convert from speed (-100 to 100) to servo command (0 to 255)
 int speed_to_command(int8_t speed) {
     float kp = 0.2;
     return (int)(kp * speed + 127);
 }
 
 // Sets motor to a specific speed with display
 void motor(uint8_t num, int cmd_value) {
     char print_speed[16];
     
     sprintf(print_speed, "M%d: %d", num, cmd_value);
     lcd_cursor(0, num == 2 ? 0 : 1);
     print_string(print_speed);
     
     set_servo(num, cmd_value);
 }
 
 // Move both motors simultaneously with left motor boost
 void move_motors(int left_target, int right_target) {
     int left_current = 127;
     int right_current = 127;
     
     // Apply boost to left motor during forward movement
     if (left_target > 127) {
         // If moving forward, add power to left motor
         left_target += LEFT_BOOST;
         
         // Make sure we don't exceed valid range
         if (left_target > 255) left_target = 255;
     }
     
     int left_steps = abs(left_target - left_current);
     int right_steps = abs(right_target - right_current);
     int total_steps = (left_steps > right_steps) ? left_steps : right_steps;
     
     int step_size = 2;
     
     float left_increment = (left_target - left_current) / (float)total_steps * step_size;
     float right_increment = (right_target - right_current) / (float)total_steps * step_size;
     
     clear_screen();
     
     for (int i = 0; i < total_steps; i += step_size) {
         left_current += left_increment;
         right_current += right_increment;
         
         if ((left_increment > 0 && left_current > left_target) ||
             (left_increment < 0 && left_current < left_target)) {
             left_current = left_target;
         }
         
         if ((right_increment > 0 && right_current > right_target) ||
             (right_increment < 0 && right_current < right_target)) {
             right_current = right_target;
         }
         
         motor(2, (int)left_current);
         motor(3, (int)right_current);
         
         _delay_ms(75);
     }
     
     // Ensure final values are set exactly
     motor(2, left_target);
     motor(3, right_target);
 }
 
 // Move robot forward with left motor boost
 void forward() {
     lcd_cursor(0, 0);
     print_string("Moving Forward");
     
     int left_forward = speed_to_command(100);
     int right_forward = speed_to_command(-100);
     
     move_motors(left_forward, right_forward);
 }
 
 // Move robot backward
 void backward() {
     lcd_cursor(0, 0);
     print_string("Moving Backward");
     
     int left_backward = speed_to_command(-100);
     int right_backward = speed_to_command(100);
     
     move_motors(left_backward, right_backward);
 }
 
 // Stop both motors
 void stop() {
     lcd_cursor(0, 0);
     print_string("Stopping");
     
     move_motors(127, 127);
 }
 
 // Display current motor values on LCD
 void display_motor_values(int left_value, int right_value) {
     clear_screen();
     char buffer[16];
     
     sprintf(buffer, "L:%d R:%d", left_value, right_value);
     lcd_cursor(0, 0);
     print_string(buffer);
 }
 
 int main(int argc, char *argv[]) {
     init();
     init_servo();
     init_lcd();
     
     stop();


     clear_screen();
     print_string("Boost Test");
     _delay_ms(1000);
     
     // Test forward motion with boost
     forward();
     _delay_ms(3000);  // Longer test to observe effect
     
     // Test stopping
     stop();
     _delay_ms(1000);
     
     // Test backward motion
     backward();
     _delay_ms(3000);
     
     // Stop at end
     stop();
     
     clear_screen();
     print_string("Test Complete");
     
     return 0;
 }