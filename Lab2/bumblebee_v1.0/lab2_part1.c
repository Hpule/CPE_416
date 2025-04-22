/**
 * Name: Hector Pule and Wilson Yu
 * Lab 2 part 1
 * Description: Control servos 2 and 3 with gradual acceleration and deceleration
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
 #define ACCEL_DELAY 50  // Delay between acceleration steps (ms)
 #define ACCEL_STEP 2    // Size of acceleration steps
 
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
 
 // Move both motors simultaneously with gradual acceleration
 void move_motors(int left_target, int right_target) {
     // Store current values (likely to be the last set values)
     static int left_current = 127;
     static int right_current = 127;
     
     // Apply boost to left motor during forward movement
     if (left_target > 127) {
         // If moving forward, add power to left motor
         left_target += LEFT_BOOST;
         
         // Make sure we don't exceed valid range
         if (left_target > 255) left_target = 255;
     }
     
     clear_screen();
     
     // Calculate required steps for smooth acceleration
     float left_steps = left_target - left_current;
     float right_steps = right_target - right_current;
     
     // Determine number of increments needed for smooth transition
     int num_steps = abs(left_steps) > abs(right_steps) ? 
                     abs(left_steps) / ACCEL_STEP : 
                     abs(right_steps) / ACCEL_STEP;
     
     // Ensure a minimum number of steps for very small changes
     if (num_steps < 5) num_steps = 5;
     
     // Calculate increment values
     float left_increment = left_steps / num_steps;
     float right_increment = right_steps / num_steps;
     
     // Gradually change speed
     for (int i = 0; i < num_steps; i++) {
         left_current += left_increment;
         right_current += right_increment;
         
         // Ensure we don't overshoot targets
         if ((left_increment > 0 && left_current > left_target) ||
             (left_increment < 0 && left_current < left_target)) {
             left_current = left_target;
         }
         
         if ((right_increment > 0 && right_current > right_target) ||
             (right_increment < 0 && right_current < right_target)) {
             right_current = right_target;
         }
         
         // Set motor values
         motor(2, (int)left_current);
         motor(3, (int)right_current);
         
         clear_screen(); 
         // Display current speed values
         char buffer1[16];
         sprintf(buffer1, "L:%d", (int)left_current);
         lcd_cursor(0, 0);
         print_string(buffer1);

         char buffer2[16]; 
         sprintf(buffer2, "R:%d", (int)right_current);
         lcd_cursor(0, 1);
         print_string(buffer2);
         
         // Delay between steps for smooth acceleration
         _delay_ms(ACCEL_DELAY);
     }
     
     // Ensure final values are set exactly
     motor(2, left_target);
     motor(3, right_target);
 }
 
 int main(int argc, char *argv[]) {
     init();
     init_servo();
     init_lcd();
     
     // Initialize motors to stopped position
     motor(2, 127);
     motor(3, 127);
          
     while (1) {
         // Moving forward with gradual acceleration
         clear_screen();
         lcd_cursor(0, 0);
         print_string("Forward");
         _delay_ms(1000);
         
         int left_forward = speed_to_command(100);
         int right_forward = speed_to_command(-100);
         move_motors(left_forward, right_forward);
         _delay_ms(3000);  // Run forward for a while
         
         // Stopping with gradual deceleration
         clear_screen();
         lcd_cursor(0, 0);
         print_string("Stopping");
         _delay_ms(1000);
         move_motors(127, 127);
         _delay_ms(1000);
         
         // Moving backward with gradual acceleration
         clear_screen();
         lcd_cursor(0, 0);
         print_string("Backward");
         _delay_ms(1000);
         
         int left_backward = speed_to_command(-100);
         int right_backward = speed_to_command(100);
         move_motors(left_backward, right_backward);
         _delay_ms(3000);
         
         // Stopping with gradual deceleration
         clear_screen();
         lcd_cursor(0, 0);
         print_string("Stopping");
         _delay_ms(1000);
         move_motors(127, 127);
         _delay_ms(1000);
     }
     return 0;
 }