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
 


 #define LR_SENSOR 0
 #define LEFT_SENSOR 2
 #define RIGHT_SENSOR 3
 
 #define LEFT_SERVO 2
 #define RIGHT_SERVO 3
 #define SERVO_MAX 157 
 #define SERVO_MIN 97
 

 #define DETECT_THRESH 45 // threshold for detecting a soda can (could increase to 50 or so)
 #define LINE_THRES 185 // threshold for black line 
 #define WHITE_AREA_THRESH_LEFT 170  // White area threshold for left sensor
 #define WHITE_AREA_THRESH_RIGHT 160 // White area threshold for right sensor
 #define EDGE_BOUNDARY_THRESH 195    // Edge/boundary detection threshold

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
   const float ki = 0.05; // integral gain
   const float kd = 0.10; // derivative gain
   
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
   int left_forward = pid_controller(-5); // test 20% so we hopefully get less overshoot
   int right_forward = pid_controller(-25);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }

  void turn_left_only() {
   // Only left wheel moves backward to turn right
   set_servo(LEFT_SERVO, MOTOR_STOP - 30);  // Left wheel backward
   set_servo(RIGHT_SERVO, MOTOR_STOP);      // Right wheel stopped
 }
 
 void right() {
   int left_forward = pid_controller(50); // test 20% so we hopefully get less overshoot 
   int right_forward = pid_controller(10);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }

  void turn_right_only() {
   // Only right wheel moves backward to turn left  
   set_servo(LEFT_SERVO, MOTOR_STOP);       // Left wheel stopped
   set_servo(RIGHT_SERVO, MOTOR_STOP - 30); // Right wheel backward
 }
 
 void stop() {
   set_servo(LEFT_SERVO, 127);
   set_servo(RIGHT_SERVO, 127);
 }

 void backward(){
     int left_forward = pid_controller(-100); // test 5% so we hopefully get less overshoot 
   int right_forward = pid_controller(100);
 
   set_servo(LEFT_SERVO, left_forward);
   set_servo(RIGHT_SERVO, right_forward);
 }
 

  // Function to check if robot has reached edge/boundary
 uint8_t check_edge_reached(uint8_t left_val, uint8_t right_val) {
    // Check if either sensor detects edge/boundary (value around 195)
    if (left_val >= EDGE_BOUNDARY_THRESH || right_val >= EDGE_BOUNDARY_THRESH) {
        return 1; // Edge detected
    }
    return 0; // Still in safe area
 }

 
 // Scanning function with state machine
 void scan() {
    static uint8_t scan_state = 0; // 0 = initial turn, 1 = wait period, 2 = active scanning
    static uint32_t scan_timer = 0;
    static uint8_t turn_direction = 1; // 1 = right, 0 = left
    
    scan_timer++; // Increment timer each call
    
    switch(scan_state) {
        case 0: // Initial turn phase
            if (turn_direction) {
                right(); // Turn right initially
            } else {
                left();  // Turn left initially
            }
            scan_timer = 0; // Reset timer
            scan_state = 1; // Move to wait state
            break;
            
        case 1: // Wait for 1 second (approximately)
            if (turn_direction) {
                right(); // Continue turning right
            } else {
                left();  // Continue turning left
            }
            
            // Wait for approximately 1 second (adjust 1000 based on your loop speed)
            if (scan_timer > 1000) {
                scan_state = 2; // Move to active scanning
                scan_timer = 0; // Reset timer for timeout
            }
            break;
            
        case 2: // Active scanning phase
            if (turn_direction) {
                right(); // Keep turning right
            } else {
                left();  // Keep turning left
            }
            
            // Optional: timeout to switch direction if nothing found
            if (scan_timer > 5000) { // 5 second timeout
                turn_direction = !turn_direction; // Switch direction
                scan_state = 0; // Restart scanning in opposite direction
                scan_timer = 0;
            }
            break;
    }
 }


 void pathing(int16_t lr_val, uint8_t left_val, uint8_t right_val) {
    static uint8_t robot_state = 0; // 0 = waiting, 1 = following, 2 = timing, 3 = correcting, 4 = backing up
    static uint32_t line_to_edge_timer = 0;
    static uint32_t backup_timer = 0;
    static uint32_t backup_duration = 0;
    static uint32_t correction_timer = 0;
    static uint8_t correction_direction = 0; // 0 = left hit first, 1 = right hit first
    static uint32_t print_counter = 0; // To limit print frequency
    
    print_counter++;
    
    switch(robot_state) {
        case 0: // Waiting for object detection
            if (print_counter % 1 == 0) { // Print every 1 cycles
                clear_screen();
                lcd_cursor(0, 0);
                print_string("0 Wait");
                _delay_ms(150);
            }
            
            if (lr_val > DETECT_THRESH) {
                robot_state = 1; // Object found, start following
                clear_screen();
                lcd_cursor(0, 0);
                print_string("0 FOUND");
                _delay_ms(150); // 1 second delay to see message
            } else {
                stop(); // Wait until object is detected
            }
            break;
            
        case 1: // Following the object
            if (print_counter % 1 == 0) {
                clear_screen();
                lcd_cursor(0, 0);
                print_string("1 Foll");
                lcd_cursor(0, 1);
                char msg[17];
                sprintf(msg, "LR:%d", lr_val);
                print_string(msg);
                _delay_ms(150);
            }
            
            if (lr_val > DETECT_THRESH) {
                forward(); // Keep following the object
            } else {
                // Object lost! Start timing from line threshold to edge
                robot_state = 2;
                line_to_edge_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("1 LOST");
                _delay_ms(150); // 1 second delay
            }
            break;
            
        case 2: // Timing from line threshold to edge boundary
            if (print_counter % 5 == 0) { // More frequent updates during timing
                clear_screen();
                lcd_cursor(0, 0);
                print_string("2 Edge");
                lcd_cursor(0, 1);
                _delay_ms(150);
            }
            
            forward(); // Keep moving forward to measure the distance
            line_to_edge_timer++;
            
            // Check which sensor hits edge first
            if (left_val >= EDGE_BOUNDARY_THRESH && right_val < EDGE_BOUNDARY_THRESH) {
                // Left sensor hit edge first - need to turn right (move right wheel only)
                correction_direction = 0; // Left hit first
                robot_state = 3; // Go to correction state
                correction_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("2L HIT");
                lcd_cursor(0, 1);
                print_string("TRN R");
                _delay_ms(150);
            }
            else if (right_val >= EDGE_BOUNDARY_THRESH && left_val < EDGE_BOUNDARY_THRESH) {
                // Right sensor hit edge first - need to turn left (move left wheel only)  
                correction_direction = 1; // Right hit first
                robot_state = 3; // Go to correction state
                correction_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("2RHIT");
                lcd_cursor(0, 1);
                print_string("Trn L");
                _delay_ms(150);
            }
            else if (left_val >= EDGE_BOUNDARY_THRESH && right_val >= EDGE_BOUNDARY_THRESH) {
                // Both sensors hit edge at same time - skip correction, go straight to backup
                backup_duration = line_to_edge_timer * 2;
                robot_state = 4; // Go directly to backing up
                backup_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("2 BE");
                _delay_ms(150);
            }
            
            // Safety timeout
            if (line_to_edge_timer > 3000) {
                backup_duration = line_to_edge_timer;
                robot_state = 4;
                backup_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("TIMEOUT!");
                lcd_cursor(0, 1);
                print_string("Force backup");
                _delay_ms(150);
            }
            break;
            
        case 3: // Correcting orientation until both sensors hit edge
            if (print_counter % 1 == 0) {
                clear_screen();
                lcd_cursor(0, 0);
                print_string("3 CRT");
                lcd_cursor(0, 1);
                _delay_ms(150);
            }
            
            correction_timer++;
            
            if (correction_direction == 0) {
                // Left hit first, turn right by moving only right wheel backward
                turn_right_only();
            } else {
                // Right hit first, turn left by moving only left wheel backward  
                turn_left_only();
            }
            
            // Continue correction until both sensors detect edge
            else if (correction_direction == 0 && left_val < EDGE_BOUNDARY_THRESH - 10) {
                // Both sensors now detect edge - start backing up
                backup_duration = line_to_edge_timer;
                robot_state = 4;
                backup_timer = 0;
                correction_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("3 CRTED");
                _delay_ms(150);
            }
            
            // Safety timeout for correction
            else if (correction_direction == 1 && right_val < EDGE_BOUNDARY_THRESH - 10) {
                backup_duration = line_to_edge_timer;
                robot_state = 4;
                backup_timer = 0;
                correction_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("3 CRT T");
                lcd_cursor(0, 1);
                print_string("Force backup");
                _delay_ms(150);
            }
            break;
            
        case 4: // Backing up for calculated duration
            if (print_counter % 1 == 0) {
                clear_screen();
                lcd_cursor(0, 0);
                print_string("4 BKUP");
                _delay_ms(150);
            }
            
            backward();
            backup_timer++;
            
            if (backup_timer >= backup_duration) {
                robot_state = 0; // Return to waiting for next object
                backup_timer = 0;
                backup_duration = 0;
                line_to_edge_timer = 0;
                clear_screen();
                lcd_cursor(0, 0);
                print_string("4BACKUP DONE!");
                lcd_cursor(0, 1);
                print_string("Reset to wait");
                _delay_ms(150); // 2 second pause before restarting
            }
            break;
    }
 }


//  void pathing(int16_t lr_val) {
//     static uint8_t robot_state = 0; // 0 = scanning, 1 = moving forward, 2 = backup
//     static uint32_t forward_timer = 0;
//       static uint32_t backup_timer = 0;

    
//     switch(robot_state) {
//         case 0: // Scanning state
//             if (lr_val > DETECT_THRESH) {
//                 // Object detected! Switch to forward movement
//                 robot_state = 1;
//                 forward_timer = 0;
//             } else {
//                 // Keep scanning
//                 scan();
//             }
//             break;
            
//         case 1: // Moving forward toward detected object
//             forward();
//             forward_timer++;

//             if (check_edge_reached(left_val, right_val)) {
//               // Reached the edge - start backing up
//               robot_state = 2;
//               backup_timer = 0;
//               break;
//             }
            
//             // Optional: move forward for a certain time, then return to scanning
//             if (forward_timer > 2000) { // Move forward for ~2 seconds
//                 robot_state = 0; // Return to scanning
//                 forward_timer = 0;
//             }
            
//             // If we lose the object while moving forward, return to scanning
//             if (lr_val <= DETECT_THRESH) {
//                 robot_state = 0; // Return to scanning                
//                 forward_timer = 0;
//             }
//             break;
//     }
//  }
 
 int main(void) { 
    initialize();

    uint8_t system_started = 0;
 
   while (1) {
    // Read all sensor values
    uint8_t lr = analog(LR_SENSOR); // get the long-range sensor value
    uint8_t left_value = analog(LEFT_SENSOR);  // get left sensor
    uint8_t right_value = analog(RIGHT_SENSOR); // get right sensor
    uint8_t button = get_btn();
     
     clear_screen();
     
    //  Display LR sensor on first line
     char lr_msg[17];
     lcd_cursor(0, 0);
     sprintf(lr_msg, "LR:%3d", lr);
     print_string(lr_msg);

     char buffer1[17];
     lcd_cursor(0, 1);
     sprintf(buffer1, "%3d %3d", (int)left_value, (int)right_value);
     print_string(buffer1);
     _delay_ms(1500);

     if (!system_started) {
      if (button){
             system_started = 1; // Start the system
             clear_screen();
             lcd_cursor(0, 0);
             print_string("SYSTEM STARTED!");
             _delay_ms(1500);
      } else{
             clear_screen();
             lcd_cursor(0, 0);
             print_string("START");
             lcd_cursor(0, 1);
             print_string("PRSS BTN");
             _delay_ms(1500);
      }
     }else{
        pathing(lr, left_value, right_value);
     }
    
    //  pathing(lr);
    pathing(lr, left_value, right_value);

     
     // Small delay to make display readable
     _delay_ms(100);
   }
 }