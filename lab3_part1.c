#include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 


 #define LR_SENSOR 0
 #define LEFT_SENSOR 2
 #define RIGHT_SENSOR 3
 
 #define LEFT_SERVO 2
 #define RIGHT_SERVO 3
 #define SERVO_MAX 157 
 #define SERVO_MIN 97
 

 #define DETECT_THRESH 40 // threshold for detecting a soda can (could increase to 50 or so)
 #define LINE_THRES 180 // threshold for black line 

 #define MOTOR_STOP 127
 
 int16_t prev_error = 0; // this is the previous error value that we will use to calculate the derivative
 int16_t prev_diff = 0;  // this is the previous difference value that we will use to calculate the derivative 

 void initialize() {
    init();
    init_servo();
    init_lcd();
    init_adc();
    
    // Initialize motors to stopped position
    set_servo(LEFT_SERVO, MOTOR_STOP);
    set_servo(RIGHT_SERVO, MOTOR_STOP);
    
    clear_screen();
    lcd_cursor(0, 0);
}
 
 
 int pid_controller(int16_t error) {
      const float kp = 0.80; // proportional gain
   const float ki = 0.01; // integral gain
   const float kd = 0.03; // derivative gain
   
   static float integral = 0.0; // accumulator for integral term
   
   // update integral (accumulate error over time)
   integral += error;
   
   // optional: implement integral windup protection
   const float integral_max = 100.0; // adjust based on your system
   if (integral > integral_max) {
       integral = integral_max;
   } else if (integral < -integral_max) {
       integral = -integral_max;
   }
 
   // calculate the PID command relative to the stop bias (127)
   float command_delta = kp * error + ki * integral + kd * (error - prev_error) + 127;

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
   int left_forward = pid_controller(40); // test 5% so we hopefully get less overshoot 
   int right_forward = pid_controller(-40);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }

void backward() {
   int left_forward = pid_controller(-40); // test 5% so we hopefully get less overshoot 
   int right_forward = pid_controller(40);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 
 void left() {
   int left_forward = pid_controller(5); // test 20% so we hopefully get less overshoot
   int right_forward = pid_controller(20);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void right() {
   int left_forward = pid_controller(20); // test 20% so we hopefully get less overshoot 
   int right_forward = pid_controller(5);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void stop() {
   set_servo(LEFT_SERVO, 127);
   set_servo(RIGHT_SERVO, 127);
 }
 
 void check_sensors(uint8_t left_value, uint8_t right_value) {
    // FIXED: Check both sensors first (most critical case)
    if (left_value > LINE_THRES && right_value > LINE_THRES) {
        // Both sensors detecting line - back up straight
        clear_screen();
        lcd_cursor(0, 0);
        print_string("BOTH LINE!");
        _delay_ms(400);
        stop();
        _delay_ms(300);
        backward();
        _delay_ms(300);
    }
    // if the left sensor is detecting a line, turn right
    else if (left_value > LINE_THRES) {
        clear_screen();
        lcd_cursor(0, 0);
        print_string("LEFT LINE!");
        stop();
        _delay_ms(300);
        backward();
        _delay_ms(300);
        right();
        _delay_ms(300);

    }
    // if the right sensor is detecting a line, turn left
    else if (right_value > LINE_THRES) {
        clear_screen();
        lcd_cursor(0, 0);
        print_string("RIGHT LINE!");
        stop();
        _delay_ms(300);
        backward();
        _delay_ms(300);
        left();
        _delay_ms(300);

    }
 }

 // FIXED: Function for scanning with sensor checking
 void scan(uint8_t left_value, uint8_t right_value) {
     // Check sensors before any movement
     check_sensors(left_value, right_value);
     
     // Only continue scanning if no line detected
     if (left_value <= LINE_THRES && right_value <= LINE_THRES) {
         right();
         _delay_ms(100); // delay to allow for scanning motion
         
         // Check sensors again after turning right
         check_sensors(left_value, right_value);
         
         // Only continue if still safe
         if (left_value <= LINE_THRES && right_value <= LINE_THRES) {
             left();
             _delay_ms(300); // delay to allow for scanning motion
         }

         check_sensors(left_value, right_value);
         forward();
        _delay_ms(150);

         check_sensors(left_value, right_value);
     }
 }

 // FIXED: Simplified pathing logic
 void pathing(int16_t lr_val, uint8_t left_value, uint8_t right_value) {
    // Always check sensors first for safety
    check_sensors(left_value, right_value);
    
    // Only proceed with normal behavior if no line detected
    if (left_value <= LINE_THRES && right_value <= LINE_THRES) {
        // if we can find the can, move towards it 
        if (lr_val > DETECT_THRESH) {
           clear_screen();
           lcd_cursor(0, 0);
           print_string("CAN FOUND!");
           lcd_cursor(0, 1);
           char msg[17];
           sprintf(msg, "LR:%d", lr_val);
           print_string(msg);
           _delay_ms(100);
           forward(); // move forward if the sensor detects a can

           check_sensors(left_value, right_value);
        }
        // otherwise we want to scan for it
        else {
           clear_screen();
           lcd_cursor(0, 0);
           print_string("SCANNING...");
           lcd_cursor(0, 1);
           char msg[17];
           sprintf(msg, "%d<%d", lr_val, DETECT_THRESH);
           print_string(msg);
           _delay_ms(100);
           scan(left_value, right_value);
        }
    }
    
    _delay_ms(50); // Small delay for stability
 }


 int main(void) { 
initialize();
    
    // Wait for first button press to start
    uint8_t system_started = 0;
 
   while (1) {
     // Read all sensor values
     uint8_t lr = analog(LR_SENSOR); // get the long-range sensor value
     uint8_t left_value = analog(LEFT_SENSOR);  // get left sensor
     uint8_t right_value = analog(RIGHT_SENSOR); // get right sensor
     uint8_t button = get_btn(); // Read button state
     
     if (!system_started) {
         // System hasn't started yet - wait for button press
         if (button) {
             system_started = 1; // Start the system
             clear_screen();
             lcd_cursor(0, 0);
             print_string("STARTED!");
             _delay_ms(400);
         } else {
             // Show sensor readings while waiting for button
             clear_screen();
             lcd_cursor(0, 1);
             char lr_msg[17];
             sprintf(lr_msg, "LR:%3d", lr);
             print_string(lr_msg);
             
             lcd_cursor(0, 0);
             print_string("PRESS BUTTON");
             _delay_ms(300);
         }
     } else {
         // System started - run normal tracking behavior
         
         // Clear the display area first to prevent text overlap
         clear_screen();
         
         // Display LR sensor on first line
         char lr_msg[17];
         lcd_cursor(0, 0);
         sprintf(lr_msg, "LR:%3d", lr);
         print_string(lr_msg);

         // Display Left and Right sensors on second line
         char buffer1[17];
         lcd_cursor(0, 1);
         sprintf(buffer1, "L:%3d R:%3d", (int)left_value, (int)right_value);
         print_string(buffer1);
         _delay_ms(100);
        
         pathing(lr, left_value, right_value);
     }
     
     // Small delay to make display readable
     _delay_ms(50);
   }
 }