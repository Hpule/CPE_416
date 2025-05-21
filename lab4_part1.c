/**
 * Name: Hector Pule and Wilson Yu
 * Lab 4 Part 1
 * Description: Wheel encoder for square & bowtie patterns
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define BASE_SPEED 40
#define TURN_SPEED 26
#define MOTOR_STOP 127          // Servo value for stopped position

// Square
#define STRAIGHT_TICKS 85
#define TURN_TICKS 16

// Bowtie 
#define DIAGONAL_TICKS 110 // rounded up b/c it was 125.86 but could also be 125  
#define BOW_TURN_TICKS 96

#define LEFT_SERVO 2
#define RIGHT_SERVO 3

#define RIGHT_DIG 4
#define LEFT_DIG 5

volatile uint16_t left_encoder = 0; 
volatile uint16_t right_encoder = 0; 
uint8_t pattern = 0; // 0 = square, 1 = bowtie


void stop(void); 
void init_encoder(void); 
void move_straight(uint16_t ticks);
void turn_right_forward(uint16_t ticks);
void turn_left_forward(uint16_t ticks);
void drive_square(void); 
void drive_bowtie(); 
void turn_angle(uint8_t angle); 
void display(void); 


void init_encoder() {
    EIMSK |= _BV(PCIE1) | _BV(PCIE0);
    PCMSK1 |= _BV(PCINT13); // Digital 5
    PCMSK0 |= _BV(PCINT6);  // Digital 4
    PORTE |= _BV(PE6);      // Pullup
    PORTB |= _BV(PB5);      // Pullup
}

ISR(PCINT0_vect) {
   left_encoder++;
}

ISR(PCINT1_vect) {
   right_encoder++;
}


void display(){
    clear_screen();
    lcd_cursor(0, 0);
    print_string("L:");
    print_num(left_encoder);
    lcd_cursor(0, 1);
    print_string("R:");
    print_num(right_encoder);
}


void move_straight(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP + BASE_SPEED);
    set_servo(RIGHT_SERVO, MOTOR_STOP - BASE_SPEED);
    
    while (left_encoder < ticks && right_encoder < ticks) {
        display(); 
        _delay_ms(100);
    }    
    stop(); 
}

void turn_right_forward(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP + TURN_SPEED);
    set_servo(RIGHT_SERVO, MOTOR_STOP + TURN_SPEED);
    
    while (right_encoder < ticks) { // For some reason left works...
        display(); 
        _delay_ms(100);
    }
    
    stop(); 
}

void turn_left_forward(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP - TURN_SPEED);
    set_servo(RIGHT_SERVO, MOTOR_STOP - TURN_SPEED);

    while (left_encoder < ticks) { // For some reason left works...
        display(); 
        _delay_ms(100);
    }
    
    stop();
}


void stop(){
    set_servo(LEFT_SERVO, MOTOR_STOP);
    set_servo(RIGHT_SERVO, MOTOR_STOP);
}

void drive_square() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Square Pattern");
    _delay_ms(1000);
    

    move_straight(100);
    stop();
    _delay_ms(1000);     
    
    turn_right_forward(24);
    stop();
    _delay_ms(1000);     

    move_straight(100 + 5);
    stop();
    _delay_ms(1000);     
    
    turn_right_forward(24);
    stop();
    _delay_ms(1000);     

    move_straight(100);
    stop();
    _delay_ms(1000);     
    
    turn_right_forward(24);
    stop();
    _delay_ms(1000);

    
    
}


void drive_bowtie() {
    // calcaulated error and diagnol 
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Bowtie");
    _delay_ms(1000);
    
    // logic to drive the bowtie 
    move_straight(100);  // goes up the line then turns right on the first corner 
    stop();
    _delay_ms(1000);

    turn_right_forward(38);
    stop();
    _delay_ms(1000);


    move_straight(150); // moves diagonal across to the second corner, then turns left 
    stop();
    _delay_ms(1000);


    turn_left_forward(25);
    stop();
    _delay_ms(1000);

    move_straight(100); // Second up  then turns left 
    stop();
    _delay_ms(1000);


    turn_left_forward(24);
    stop();
    _delay_ms(1000);

    move_straight(150); // moves diagonal across to the second corner, then turns left 
    stop();
    _delay_ms(1000);

    // move_straight(130); // moves diagonal across to the second corner, then turns left 
    // stop();
    // _delay_ms(1000);


    // turn_right_forward(32); 
    // stop();
    // _delay_ms(1000);
    

    clear_screen();
    lcd_cursor(0, 0);
    print_string("Bowtie Fin");
    _delay_ms(2000);
}

void wheel_calibration() {
    set_servo(LEFT_SERVO, 127);
    set_servo(RIGHT_SERVO, 127);
}

int main(void) {
    init();
    init_lcd(); 
    init_servo(); 
    init_encoder();
    stop(); 

    digital(LEFT_DIG); 
    digital_dir(LEFT_DIG, 0); 
    digital(RIGHT_DIG); 
    digital_dir(RIGHT_DIG, 0); 
    
    bool pattern_state = false; // false = square, true = bowtie
    bool button_pressed = false; // Track if button was pressed
    
    while(1) {
        // Show current pattern
        clear_screen();
        lcd_cursor(0, 0);
        print_string("Press to toggle");
        lcd_cursor(0, 1);
        print_string(pattern_state ? "Bowtie" : "Square");
        
        // Debounced button handling
        if (get_btn()) {
            if (!button_pressed) {
                button_pressed = true;
                // Toggle pattern state only once per button press
                pattern_state = !pattern_state;
                
                // Display change confirmation
                clear_screen();
                lcd_cursor(0, 0);
                print_string("Mode changed to:");
                lcd_cursor(0, 1);
                print_string(pattern_state ? "Bowtie" : "Square");
                _delay_ms(1000); // Show confirmation message
            }
        } else {
            button_pressed = false;
        }
        
        // Run the selected pattern when button is released
        if (!get_btn() && button_pressed == false) {
            if (!pattern_state) {
                drive_bowtie();
            } else {
                drive_square();
            }
            
            // After pattern completes, wait for a moment
            _delay_ms(2000);
        }
        
        _delay_ms(100); // General loop delay
    }
    
    return 0;
}