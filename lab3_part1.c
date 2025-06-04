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
 

 #define DETECT_THRESH 45 // threshold for detecting a soda can (could increase to 50 or so)
 #define LINE_THRES 185 // threshold for black line 

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
   int left_forward = pid_controller(100); // test 5% so we hopefully get less overshoot 
   int right_forward = pid_controller(-100);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void left() {
   int left_forward = pid_controller(10); // test 20% so we hopefully get less overshoot
   int right_forward = pid_controller(50);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void right() {
   int left_forward = pid_controller(50); // test 20% so we hopefully get less overshoot 
   int right_forward = pid_controller(10);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 
 void stop() {
   set_servo(LEFT_SERVO, 127);
   set_servo(RIGHT_SERVO, 127);
 }
 
 // a function used for scanning for cans
 void scan() {
     right();
      _delay_ms(100); // delay to allow for scanning motion
      left();
      left();
      _delay_ms(100); // delay to allow for scanning motion
 }

 void pathing(int16_t lr_val, uint16_t left_value, uint16_t right_value) {
  

  /* Bound Handling Code */
  // if the left sensor is detecting a line, turn right
    if (left_value > LINE_THRES) {
        right();
    }
    // if the right sensor is detecting a line, turn left
    else if (right_value > LINE_THRES) {
        left();
    }
    // if both sensors are detecting a line, stop
    else if (left_value > LINE_THRES && right_value > LINE_THRES) {
        stop(); // probably just have it choose a single behavior (left or right turn)
    }
    // if we are within bounds 
    else {
      // if we can find the can, move towards it 
      if (lr_val > DETECT_THRESH) {
         forward(); // move forward if the sensor detects a can (may want a can buffer)
      }
      // otherwise we want to scan for it
      else {
         scan();
      }
    }
    
    // might want a scanning motion to detect the can 
    // accumulate can detections? 
 }


 int main(void) { 
    initialize();
 
   while (1) {
     // Read all sensor values
     uint8_t lr = analog(LR_SENSOR); // get the long-range sensor value
     uint8_t left_value = analog(LEFT_SENSOR);  // get left sensor
     uint8_t right_value = analog(RIGHT_SENSOR); // get right sensor
     
     // Clear the display area first to prevent text overlap
     clear_screen();
     
     // Display LR sensor on first line
     char lr_msg[17];
     lcd_cursor(0, 0);
     sprintf(lr_msg, "LR:%3d", lr);
     print_string(lr_msg);

    //  // Display Left and Right sensors on second line
    //  char buffer1[17];
    //  lcd_cursor(0, 1);
    //  sprintf(buffer1, "L:%3d R:%3d", (int)left_value, (int)right_value);
    //  print_string(buffer1);
    
     pathing(lr, left_value, right_value);
     
     // Small delay to make display readable
     _delay_ms(100);
   }
 }