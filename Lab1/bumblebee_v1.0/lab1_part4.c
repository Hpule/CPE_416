/**
 * Name: Hector Pule and Wilson Yu
 * Lab 1 part 4
 * Description: Controlling "416" on LCD display, which is moved by tilting the board. 
 * 
 */

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

char text[] = "416"; // Text to display

// Function to update position based on accelerometer readings
void update_position(int* row, int* col) {
    // Define thresholds
    const int TOP_THRESHOLD = 190;
    const int BOTTOM_THRESHOLD = 60;
    
    // Get accelerometer readings
    int16_t x_accel = get_accel_x();
    int16_t y_accel = get_accel_y();
    
    // Update row based on X accelerometer
    if (x_accel >= TOP_THRESHOLD && x_accel != 0) {
        *row = 1;
    }
    else if (x_accel <= BOTTOM_THRESHOLD && x_accel != 0) {
        *row = 0;
    }
    
    // Calculate maximum column to keep text on screen
    int max_col = 8 - strlen(text);
    
    // Update column based on Y accelerometer
    if(y_accel >= TOP_THRESHOLD && ((y_accel != 0) && (y_accel != 1))) {
        (*col)++; // Move right
        if(*col > max_col) *col = max_col; // Don't go off screen
    } 
    else if(y_accel <= BOTTOM_THRESHOLD && ((y_accel != 0) && (y_accel != 1))) {
        (*col)--; // Move left
        if(*col < 0) *col = 0; // Don't go off screen
    }
}

int main(void) {
    init();  // Initialize board hardware
    
    // Initial position
    int current_row = 0; // Start at top row
    int current_col = 2; // Start near middle
    
    while(1) {
        // Update position based on accelerometer readings
        update_position(&current_row, &current_col);
        
        // Clear screen and display text
        clear_screen();
        lcd_cursor(current_col, current_row); 
        print_string(text);
        
        // Delay for smooth movement
        _delay_ms(150);
    }
    
    return 0;
}