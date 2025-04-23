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
 
 void fear(int16_t light_diff) { // gollum hates the sun
    if (light_diff > 0) {
        // Left sensor has more light, move left motor
        set_servo(LEFT_MOTOR, MOTOR_STOP + MOTOR_SPEED); // positive motor speed moves left forward
        set_servo(RIGHT_MOTOR, MOTOR_STOP);
    } 
    else {
        // Right sensor has more light, move right motor
        set_servo(LEFT_MOTOR, MOTOR_STOP);
        set_servo(RIGHT_MOTOR, MOTOR_STOP - MOTOR_SPEED); // negative motor speed moves right forward
    }
 }

 void aggro(int16_t light_diff) { // go towards the light my son 
    if (light_diff > 0) {
        // Left sensor has more light, move right motor
        set_servo(LEFT_MOTOR, MOTOR_STOP); 
        set_servo(RIGHT_MOTOR, MOTOR_STOP - MOTOR_SPEED);
    } 
    else {
        // Right sensor has more light, move left motor
        set_servo(LEFT_MOTOR, MOTOR_STOP + MOTOR_SPEED); 
        set_servo(RIGHT_MOTOR, MOTOR_STOP);
    }
 }
 
 
 void braitenburg(bool change) { // fear (2a) & aggression (2b)  
     // Read raw light sensor values
     uint8_t left_value = analog(LEFT_SENSOR);
     uint8_t right_value = analog(RIGHT_SENSOR);
     
     // Calculate absolute difference between sensors
     int16_t light_diff = left_value - right_value;
     int16_t abs_diff;  

     if (light_diff < 0) {
         abs_diff = -light_diff;
     }

     else {
        abs_diff = light_diff;
     }
     
     // Display sensor values (voltage readings)
    //  char left_msg[17];
    //  char right_msg[17];
     
    //  lcd_cursor(0, 0);
    //  sprintf(left_msg, "L:%3d", left_value);
    //  print_string(left_msg);
     
    //  lcd_cursor(0, 1);
    //  sprintf(right_msg, "R:%3d", right_value);
    //  print_string(right_msg);
     
     // Only move if the difference is significant
     if (abs_diff >= LIGHT_DIFF_THRESHOLD) {
         if (change) { // this is fear 
            fear(light_diff);
            clear_screen();
            print_string("Scary");
        }
        else { // this is aggression
            aggro(light_diff);
            clear_screen();
            print_string("Kill");
        } 
    }

     else {
         // Difference not significant, stay still
         set_servo(LEFT_MOTOR, MOTOR_STOP);
         set_servo(RIGHT_MOTOR, MOTOR_STOP);
     }
 }



 
 int main(void) {
     // Initialize everything
     initialize();
     int change = false; 
     
     // Main loop
     while (1) {

        // change state for mode swtiching if button is pressed 
         if (get_btn() == 1) {
             change = !change;
         } 

         braitenburg(change);
         
         // Small delay
         _delay_ms(100);

         clear_screen(); 
     }
     
     return 0;
 }