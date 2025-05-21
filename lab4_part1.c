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

#define BASE_SPEED 50
#define TURN_SPEED 30
#define MOTOR_STOP 127          // Servo value for stopped position

// Square
#define STRAIGHT_TICKS 89
#define TURN_TICKS 50

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
void turn_right_backward(uint16_t ticks);
void turn_left_backward(uint16_t ticks);
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
    set_servo(RIGHT_SERVO, MOTOR_STOP - BASE_SPEED + 30);
    
    while (left_encoder < ticks || right_encoder < ticks) {
        display(); 
        _delay_ms(100);
    }    
    stop(); 
}

void turn_right_forward(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP + TURN_SPEED);
    
    while (right_encoder < ticks) { // For some reason left works...
        display(); 
        _delay_ms(100);
    }
    
    stop(); 
}

void turn_left_forward(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(RIGHT_SERVO, MOTOR_STOP - TURN_SPEED);

    while (left_encoder < ticks) { // For some reason left works...
        display(); 
        _delay_ms(100);
    }
    
    stop();
}

void turn_right_backward(uint16_t ticks){
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP - TURN_SPEED);
    
    while (right_encoder < ticks) { // For some reason left works...
        display(); 
        _delay_ms(100);
    }
    
    stop(); 
}

void turn_left_backward(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(RIGHT_SERVO, MOTOR_STOP + TURN_SPEED);

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
    
    for (int i = 0; i < 4; i++) {
        clear_screen();
        lcd_cursor(0, 0);
        print_string("Straight");
        _delay_ms(1000); 
        move_straight(STRAIGHT_TICKS);
        stop();

        if(i < 3){
            clear_screen();
            lcd_cursor(0, 0);
            print_string("Left");
            _delay_ms(1000);
            turn_right_forward(TURN_TICKS);
            stop();
        }
    }
}


void drive_bowtie() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Bowtie");
    _delay_ms(1000);
    
    // logic to drive the bowtie 
    move_straight(STRAIGHT_TICKS);  // goes up the line then turns right on the first corner 
    stop();
    _delay_ms(1000);

    turn_right_forward(75); 
    stop();
    _delay_ms(1000);


    move_straight(105); // moves diagonal across to the second corner, then turns left 
    stop();
    _delay_ms(1000);


    turn_left_forward(40);
    stop();
    _delay_ms(1000);
 
 
    turn_right_backward(40);
    stop();
    _delay_ms(1000);


    move_straight(94); // moves diagonal across to the third corner, then turns left 
    stop();
    _delay_ms(1000);


    turn_left_forward(40);
    stop();
    _delay_ms(1000);
 
 
    turn_right_backward(45);
    stop();
    _delay_ms(1000);


    move_straight(160); // moves diagonal across to the second corner, then turns left 
    stop();
    _delay_ms(1000);

    // move_straight(DIAGONAL_TICKS + 20); // moves diagonal across to the second corner, then turns left 
    // turn_right_forward(BOW_TURN_TICKS); 
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
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Press to toggle");
    lcd_cursor(0, 1);
    print_string("Square");
    
    while(1) {
        // Show current pattern
        bool state = false; 
        clear_screen();
        lcd_cursor(0, 0);
        
        print_string("Waiting");

        // If the button is pressed, change the state 
        if (get_btn()) { 
            state = !state;
        }

        // Driving mode selector (based on button press)
        if (!state) {
            drive_square(); 
        }
        else {
            drive_bowtie();
        }
        _delay_ms(100);
    }
    
    return 0;
}