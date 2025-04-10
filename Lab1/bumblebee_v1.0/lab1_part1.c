/**
 * Name: Hector Pule and Wilson Yu
 * Lab 1 part 1
 * Description: Fading led for onboard LED 0 and LED 1, using varied _delay_ms() function
 * 
 */

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Create delay loops with fixed values
void delay_fixed(uint8_t brightness) {
    // Use a simple loop for variable delay
    for (uint8_t j = 0; j < brightness; j++) {
        _delay_us(1);
    }
}

// Set LED brightness using PWM
void set_led_brightness(uint8_t pin, uint8_t brightness) {
    for (uint8_t k = 0; k < 20; k++) {
        led_on(pin);
        delay_fixed(brightness);
        led_off(pin);
        delay_fixed(100 - brightness);
    }
}

// Fade LED on then off
void fade_led(uint8_t pin) {
    // Fade LED on
    for (uint8_t i = 0; i < 100; i += 2) {
        set_led_brightness(pin, i);
        _delay_ms(5); // Small delay between brightness steps
    }
    
    // Fade LED off
    for (uint8_t i = 100; i > 0; i -= 2) {
        set_led_brightness(pin, i);
        _delay_ms(5); // Small delay between brightness steps
    }
}

int main(void) {
    init();  // Initialize board hardware
    
    while(1) {
        fade_led(LED0_PIN);  // Fade LED0 on and off
        fade_led(LED1_PIN);  // Fade LED1 on and off
    }
    
    return 0;
}