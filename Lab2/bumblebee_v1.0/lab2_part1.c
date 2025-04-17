/**
 * Name: Hector Pule and Wilson Yu
 * Lab 2 part 1
 * Description: Control servos 2 and 3 with left motor power boost
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include "globals.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 
 void motor(uint8_t num, int8_t speed) {
    float kp = 0.3; 
    int full_speed = (kp + 127); 
    int curr_speed = 0; 
    char print_speed = (char)malloc(4); // allocate memory for motor speed string

    for (int i = 0; i < speed; i++) {

        if (full_speed <= 157 && full_speed > 127 && curr_speed < 157) { 
            curr_speed = curr_speed + 10; // make it faster forward if we are in the range between 0 and 100 
        }
        else if (curr_speed >= 157) {  // logic to handle if the speed exceeds 157 
            curr_speed = 157; 
        } 

        if (full_speed >= 97 && full_speed < 127 && curr_speed > 97) {
            curr_speed = curr_speed - 10; // make it faster in reverse if we are in the range between 0 and -100
        } 
        else if (curr_speed <= 97) { // logic to handle if the speed exceeds 97
            curr_speed = 97;
        }

        else {
            curr_speed = 127; // if it's neither accelerating forwards or backwards, set it to stop (127)
        }

        clear_screen();
        sprintf(print_speed, "%d", curr_speed);
        print_string(print_speed);
        set_servo(num, curr_speed);
    }
}

void forward() { // moves Harabot forward
    motor(2, 100);
    motor(3, -100);
}

void backward() { // moves Harabot backward
    motor(2, -100);
    motor(3, 100);
}

void stop() { // stops Harabot 
    motor(2, 127);
    motor(3, 127);
}

int main(int arc, charargv[]) {
    init(); // initialization stuff 
    init_servo();
    init_lcd();

    // gradually spins the motors to full speed forward
    forward();
    _delay_ms(1000); // wait for 1 second 

    // gradually slows the motors to a stop
    stop();
    _delay_ms(1000); // wait for 1 second

    // does the same in the reverse motor direction and continuously repeats
    backward();
    _delay_ms(1000); // wait for 1 second

    return 0; 
}