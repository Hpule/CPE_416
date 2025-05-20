/**
 * Name: Hector Pule and Wilson Yu
 * Lab 4 Part 1
 * Description: Wheel encoder for square & bowtie patterns
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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
 #define DIAGONAL_TICKS 126 // rounded up b/c it was 125.86 but could also be 125  

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
void turn_right(uint16_t ticks);
void turn_left(uint16_t ticks);
void drive_square(void); 
void drive_bowtie(); 
void turn_angle(uint8_t angle); 


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


void move_straight(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, MOTOR_STOP + BASE_SPEED + 5);
    set_servo(RIGHT_SERVO, MOTOR_STOP - BASE_SPEED);
    
    while (left_encoder < ticks || right_encoder < ticks) {
        clear_screen();
        lcd_cursor(0, 0);
        print_string("L:");
        print_num(left_encoder);
        lcd_cursor(0, 1);
        print_string("R:");
        print_num(right_encoder);
        _delay_ms(100);
    }    
    stop(); 
}

void turn_right(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(LEFT_SERVO, -TURN_SPEED);
    
    while (right_encoder < ticks) { // For some reason left works...
        clear_screen();
        lcd_cursor(0, 0);
        print_string("L:");
        print_num(left_encoder);
        lcd_cursor(0, 1);
        print_string("R:");
        print_num(right_encoder);
        _delay_ms(100);
    }
    
    set_motor(0, 0);
    set_motor(1, 0);
}

void turn_left(uint16_t ticks) {
    left_encoder = 0;
    right_encoder = 0;
    set_servo(RIGHT_SERVO, TURN_SPEED);
    
    while (left_encoder < ticks) { // For some reason left works...
        clear_screen();
        lcd_cursor(0, 0);
        print_string("L:");
        print_num(left_encoder);
        lcd_cursor(0, 1);
        print_string("R:");
        print_num(right_encoder);
        _delay_ms(100);
    }
    
    set_motor(0, 0);
    set_motor(1, 0);
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
            turn_right(TURN_TICKS);
            stop();
        }
    }
}


// void turn_angle(uint8_t angle){

//     print_string("AGL");
//     _delay_ms (3000);
//     if (angle == 0) {
//         return; // No turn needed
//     }

//     // Calculate ticks based on angle (scale SMALL_TEN for the angle)
//     uint16_t turn_ticks = (abs(angle)  / 10);  // Fixed: divide by 10 since SMALL_TEN is for 10 degrees
    
//     left_encoder = 0;
//     right_encoder = 0;
    
//     clear_screen();
//     lcd_cursor(0, 0);
//     if (angle < 0) {
//         print_string("TL:");
//         set_servo(LEFT_SERVO, MOTOR_STOP - SMALL_TEN);
//         set_servo(RIGHT_SERVO, MOTOR_STOP - SMALL_TEN);
//         _delay_ms(2000);
//     } else {
//         print_string("TR:");
//         set_servo(LEFT_SERVO, MOTOR_STOP + SMALL_TEN);
//         set_servo(RIGHT_SERVO, MOTOR_STOP + SMALL_TEN);
//         _delay_ms(2000);
    // }

    // print_num(abs(angle));
    
    // while (left_encoder < turn_ticks && right_encoder < turn_ticks) {
    //     clear_screen();
    //     lcd_cursor(0, 0);
    //     print_string("L:");
    //     print_num(left_encoder);
    //     print_string(" R:");
    //     print_num(right_encoder);
    //     lcd_cursor(0, 1);
    //     print_string("T: ");
    //     print_num(turn_ticks);
    //     _delay_ms(100);  // Changed from 1000ms to 100ms for smoother display
    // }
    
    // stop();
    
    // // Show final turn values
    // clear_screen();
    // lcd_cursor(0, 0);
    // print_string("Turn Complete");
    // lcd_cursor(0, 1);
    // print_string("L:");
    // print_num(left_encoder);
    // print_string(" R:");
    // print_num(right_encoder);
    // _delay_ms(1000);
// }

void drive_bowtie() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Bowtie");
    _delay_ms(1000);
    
    // logic to drive the bowtie 
    move_straight(STRAIGHT_TICKS);  // goes up the line then turns right on the first corner 
    turn_right(50); 

    move_straight(DIAGONAL_TICKS); // moves diagonal across to the second corner, then turns left 
    turn_left(50);

    move_straight(STRAIGHT_TICKS); // goes up the line then turns left on the third corner
    turn_left(50);

    move_straight(DIAGONAL_TICKS);
    
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
        clear_screen();
        lcd_cursor(0, 0);
        
        print_string("Waiting");
        
        // Wait for button press
        if (get_btn()) {
            drive_bowtie(); 
        }
        
        _delay_ms(100);
    }
    
    return 0;
}