/**
 * Name: Hector Pule and Wilson Yu
 * Lab 1 part 3
 * Description: Simple one-person pong game. Press on button at either edge LED to keep playing. 
 *              If pressed to early or too late, game over. Displays final score after loss.
 * 
 */

#include "globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Define LED pins
#define LED_START 7 // P1 and P2 represent the bouncing back and forth (1st and 2nd stage)
#define LED_END 3
#define TOTAL_LEDS 5

void display_final_time(int delay_time); 

bool game_finished = false;

int pong_game(){
    // Initialize variables
    int current_led = LED_START;
    int delay_time = 500; // Start with 500ms delay
    bool moving_down = true; // Moving from LED_START (7) down to LED_END (3)
    bool game_over = false;
    
    // Set all LEDs as outputs and turn them off
    for (int i = LED_START; i >= LED_END; i--) {
        digital_dir(i, 1);  // Set as output
        digital_out(i, 0);  // Turn off
    }
    
    // Clear screen
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Pong Game!");
    
    while (!game_over) {
        // Turn off all LEDs
        for (int i = LED_START; i >= LED_END; i--) {
            digital_out(i, 0);
        }
        
        // Turn on current LED
        digital_out(current_led, 1);
        
        // Wait and check for button press
        bool button_pressed = false;
        
        // Split the delay into smaller chunks to check button more frequently
        for (int i = 0; i < delay_time / 10; i++) {
            if (get_btn()) {
                button_pressed = true;
                break;
            }
            _delay_ms(10);
        }
        
        // Check if button was pressed
        if (button_pressed) {
            // If button pressed at edge LEDs with correct direction, it's correct
            if ((current_led == LED_START && !moving_down) || (current_led == LED_END && moving_down)) {
                // Correct timing - change direction and speed up
                moving_down = !moving_down;
                if (delay_time > 100) {
                    delay_time -= 20;
                }
                
                // Debounce
                while (get_btn()) {
                    _delay_ms(10);
                }
                _delay_ms(100);
            } else {
                // Wrong timing - game over
                game_over = true;
                game_finished = true;
            }
        } 
        // If no button press at edge, game over
        else if ((current_led == LED_START && !moving_down) || (current_led == LED_END && moving_down)) {
            game_over = true;
            game_finished = true; 
        }
        
        // If game not over, move to next LED
        if (!game_over) {
            if (moving_down) {
                current_led--;
            } else {
                current_led++;
            }
        }
    }
    
    // Turn off all LEDs when game is over
    for (int i = LED_START; i >= LED_END; i--) {
        digital_out(i, 0);
    }
    return delay_time;
}

void display_final_time(int delay_time) {
    char time_str[17]; // Buffer for the converted integer
    sprintf(time_str, "%d ms", delay_time); // Convert integer to string
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Final Time:");
    lcd_cursor(0, 1); 
    print_string(time_str); // Pass the string, not the integer
    
    // Keep displaying until button press
    while (!get_btn()) {
        _delay_ms(100);
    }
    _delay_ms(200); // Debounce
}

int main(void) {
    init();  // Initialize board hardware
    

    // Set all LEDs as outputs and turn them off
    for (int i = LED_START; i >= LED_END; i--) {
        digital_dir(i, 1);  // Set as output
        digital_out(i, 0);  // Turn off
    }
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("LED Test");


    while(1) {
        int delay = pong_game();
        display_final_time(delay);
        _delay_ms(5000);
    }
     
        // add logic for when we lose and have to press button to start again 
    
    return 0;
}