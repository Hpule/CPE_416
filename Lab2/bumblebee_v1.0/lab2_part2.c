/**
 * Name: Hector Pule and Wilson Yu
 * Lab 2 part 2
 * Description: Braitenberg vehicle - Fear behavior using light difference
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 
 // Motor pins
 #define LEFT_MOTOR 2   // Using servo 2 for left motor
 #define RIGHT_MOTOR 3  // Using servo 3 for right motor
 
 // Sensor pins
 #define LEFT_SENSOR 0  // Analog pin 0 for left light sensor
 #define RIGHT_SENSOR 1 // Analog pin 1 for right light sensor
 
 // Thresholds and parameters
 #define LIGHT_DIFF_THRESHOLD 50  // Minimum difference to trigger motor movement
 #define MOTOR_STOP 127          // Servo value for stopped position
 #define MOTOR_SPEED 30          // Speed to move when threshold is exceeded




 // Function to initialize the system
 void initialize() {
     init();
     init_servo();
     init_lcd();
     init_adc();
     
     // Initialize motors to stopped position
     set_servo(LEFT_MOTOR, MOTOR_STOP);
     set_servo(RIGHT_MOTOR, MOTOR_STOP);
     
     clear_screen();
     lcd_cursor(0, 0);
 }

 void fear(int16_t light_diff){
    lcd_cursor(0, 0);
    print_string("Fear Mode");

        // For fear behavior: move away from higher light intensity
        if (light_diff > 0) {
            // Left sensor has more light, move left motor
            set_servo(LEFT_MOTOR, MOTOR_STOP + MOTOR_SPEED);
            set_servo(RIGHT_MOTOR, MOTOR_STOP);
        } 
        else {
            // Right sensor has more light, move right motor
            set_servo(RIGHT_MOTOR, MOTOR_STOP - MOTOR_SPEED);
            set_servo(LEFT_MOTOR, MOTOR_STOP);
        }
 }
 
 void aggression(int16_t light_diff){

    lcd_cursor(0, 0);
    print_string("Agg. Mode");

    // For fear behavior: move away from higher light intensity
    if (light_diff > 0) {
        // Left sensor has more light, move right motor
        set_servo(LEFT_MOTOR, MOTOR_STOP + MOTOR_SPEED);
        set_servo(RIGHT_MOTOR, MOTOR_STOP);
    } 
    else {
        // Right sensor has more light, move left motor
        set_servo(RIGHT_MOTOR, MOTOR_STOP - MOTOR_SPEED);
        set_servo(LEFT_MOTOR, MOTOR_STOP);
    }
 }
 
 // Update motors based on light sensor readings (Fear behavior)
 void update_motors() {
    // Read raw light sensor values
    uint8_t left_value = analog(LEFT_SENSOR);
    uint8_t right_value = analog(RIGHT_SENSOR);
    
    // Calculate absolute difference between sensors
    int16_t light_diff = left_value - right_value;
    int16_t abs_diff = (light_diff < 0) ? -light_diff : light_diff;
    
    // Display sensor values (voltage readings)
    char left_msg[17];
    char right_msg[17];
    
    // --- Debuf Sensor Voltage Data ---- //
    lcd_cursor(0, 0);
    sprintf(left_msg, "L:%3d", left_value);
    print_string(left_msg);
    
    lcd_cursor(0, 1);
    sprintf(right_msg, "R:%3d", right_value);
    print_string(right_msg);
    
    // Only move if the difference is significant
    if (abs_diff >= LIGHT_DIFF_THRESHOLD) {

    } else {
        // Difference not significant, stay still
        set_servo(LEFT_MOTOR, MOTOR_STOP);
        set_servo(RIGHT_MOTOR, MOTOR_STOP);
    }
 }
 
 int main(void) {
     // Initialize everything
     initialize();
     
     // Main loop
     while (1) {
         // Update motors based on fear behavior
         update_motors();
         
         // Small delay
         _delay_ms(100);

         clear_screen(); 

            // Check if button is pressed to switch names
            if (get_btn()) {
                _delay_ms(10); // Debounce
                
            return;
            }
     }
     
     return 0;
 }