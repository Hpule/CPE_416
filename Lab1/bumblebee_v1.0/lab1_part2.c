/**
 * Name: Hector Pule and Wilson Yu
 * Lab 1 part 2
 * Description: Using LCD screen prints out names scrolling across the screen, to change name use botton. 
 * 
 */

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

const char name1[] = "Hector Pule"; 
const char name2[] = "Wilson Yu"; 

// Function to scroll text across the LCD
void scroll_text(const char* name) {
    int name_length = strlen(name);
    char display_buffer[17]; // 16 characters + null terminator
    
    // Continue scrolling until button is pressed
    while (1) {
        // Start scrolling from right to left
        for (int pos = 16; pos > -name_length; pos--) {
            // Fill buffer with spaces initially
            for (int i = 0; i < 16; i++) {
                display_buffer[i] = ' ';
            }
            display_buffer[16] = '\0';
            
            // Fill in the visible part of the text
            for (int i = 0; i < name_length; i++) {
                int display_pos = pos + i;
                if (display_pos >= 0 && display_pos < 16) {
                    display_buffer[display_pos] = name[i];
                }
            }
            
            // Display the current buffer
            clear_screen();
            lcd_cursor(0, 0);
            print_string(display_buffer);
            
            // Check if button is pressed to switch names
            if (get_btn()) {
                _delay_ms(200); // Debounce
                return;
            }
            
            _delay_ms(200); // Scroll speed
        }
    }
}

int main(void) {
    init();  // Initialize board hardware
    
    uint8_t show_partner = 0; // Flag to determine which name to show
    
    while(1) {
        if (show_partner) {
            scroll_text(name2);
        } else {
            scroll_text(name1);
        }
        
        // Toggle which name to show next
        show_partner = !show_partner;
    }
    
    return 0;
}