/**
 * Name: Hector Pule and Wilson Yu
 * Lab 3 part 2
 * Description: Neural network controller
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h> 
#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define MAX_INPUTS 100 // max amount of sensor values (motor commands) into the  
#define HIDDEN_NEURONS 3 // number of hidden neurons in the neural network
#define LEARN_RATE 0.5

#define STATE_PROPORTIONAL 0
#define STATE_DATA_COLLECTION 1
#define STATE_TRAINING 2
#define STATE_NEURAL_NETWORK 3

#define KP 0.2  // Proportional gain
#define KI 0.01   // Integral gain
#define KD 0.05  // Derivative gain

uint8_t current_state = STATE_PROPORTIONAL; 
char *training_data = NULL;

typedef struct motor_command { // struct for the motor command (left and right speed) 
    uint8_t left_s, right_s;
} motor_command;


typedef struct sensor_reading {
    uint8_t left, right;
} sensor_reading;

sensor_reading sensor_val[MAX_INPUTS];

int sample_count = 0; 

typedef struct nnv { // struct for the neural network variables (nnv)
    float w1, w2, w3, bias; 
}	nnv; 

// neurons in the neural network (both hidden and output layers) - might not be wise to create instances of them like this, maybe just store all the values in an array
nnv h1, h2, h3, o1, o2; 

// Initialize the weights with random values
void init_neural_network() {
    // Initialize hidden layer neurons
    h1.w1 = (float)rand() / RAND_MAX;
    h1.w2 = (float)rand() / RAND_MAX;
    h1.w3 = (float)rand() / RAND_MAX;
    h1.bias = (float)rand() / RAND_MAX;
    
    h2.w1 = (float)rand() / RAND_MAX;
    h2.w2 = (float)rand() / RAND_MAX;
    h2.w3 = (float)rand() / RAND_MAX;
    h2.bias = (float)rand() / RAND_MAX;
    
    h3.w1 = (float)rand() / RAND_MAX;
    h3.w2 = (float)rand() / RAND_MAX;
    h3.w3 = (float)rand() / RAND_MAX;
    h3.bias = (float)rand() / RAND_MAX;
    
    // Initialize output layer neurons
    o1.w1 = (float)rand() / RAND_MAX;
    o1.w2 = (float)rand() / RAND_MAX;
    o1.w3 = (float)rand() / RAND_MAX;
    o1.bias = (float)rand() / RAND_MAX;
    
    o2.w1 = (float)rand() / RAND_MAX;
    o2.w2 = (float)rand() / RAND_MAX;
    o2.w3 = (float)rand() / RAND_MAX;
    o2.bias = (float)rand() / RAND_MAX;
}


void stop() {
    set_servo(2, 127); // stop the left motor
    set_servo(3, 127); // stop the right motor
} 

float sigmoid(float x) { // squeezing function for a 0-1 output 
    float result = (1 / (1 + exp(-(x)))); // sigmoid activation function
    return result; 
}

// Function to handle button press (with debouncing)
bool check_button() {
    if (get_btn()) {
        _delay_ms(200); // Debounce
        while (get_btn()) {
            _delay_ms(10); // Wait for button release
        }
        return true;
    }
    return false;
}

motor_command compute_proportional(uint8_t left, uint8_t right) { // we will put the neural output from compute_neural_network() here
    motor_command both_commands; // initialize a motor_command struct
    float kp = 0.5; // proportional gain
    float left_command = kp * (left) + 127; // equation with only proportional 
    float right_command = kp * (right) + 127; // equation with only proportional

    both_commands.left_s = left_command; // assign the left command to the struct
    both_commands.right_s = right_command; // assign the right command to the struct

    return both_commands; // return the left and right command
}

float normalize(uint8_t value) {
    float result = (value / 100.0);
    if (result < 0.0) {
        return 0.0;
    } else if (result > 1.0) {
        return 1.0;
    } else {
        return result;
    }
}

float denormalize(float value) {
    float result = (value * 100.0);
    if (result < 0.0) {
        return 0.0;
    } else if (result > 100.0) {
        return 100.0;
    } else {
        return result;
    }
}

uint16_t adjust_training_iterations() {
    uint16_t iterations = 100; // Default starting value
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Set Iterations:");
    lcd_cursor(0, 1);
    print_string("Tilt to adjust");
    _delay_ms(1000);
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("<=-  +=>");
    
    while(1) {
        uint8_t accel_y = get_accel_y();
        uint8_t accel_x = get_accel_x();
        
        if (accel_y > 30 && accel_y < 70) {iterations--; _delay_ms(100);}        // Tilted left - decrease by 1

        if (accel_y > 200 && accel_y < 250) {iterations++; _delay_ms(100);}        // Tilted right - increase by 1
        
        if (accel_x > 200 && accel_x < 230) {        // Tilted down - decrease by 10
            if (iterations >= 10) {
                iterations -= 10;
            } else {
                iterations = 0;
            }
            _delay_ms(100);
        }
        
        if (accel_x > 20 && accel_x < 70) {        // Tilted up - increase by 10
            iterations += 10;
            _delay_ms(100);
        }
        
        if (iterations < 0) {iterations = 0;}
        
        // Display current iteration count
        lcd_cursor(0, 1);
        print_num(iterations);
        print_string("       "); // Clear any previous digits
        
        // Check for button press to confirm selection
        if (check_button()) {
            break;
        }
        
        _delay_ms(50); // Control sensitivity
    }
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training with:");
    lcd_cursor(0, 1);
    print_num(iterations);
    print_string(" iterations");
    _delay_ms(1000);
    
    return iterations;
}


motor_command compute_neural_network(uint8_t sensor_left, uint8_t sensor_right) {
    motor_command computed_nodes;
    
    // Normalize inputs
    float curr_left = normalize(sensor_left);
    float curr_right = normalize(sensor_right);

    // Calculate hidden layer activations
    float h1_net = ((curr_left * h1.w1) + (curr_right * h1.w2) + h1.bias);
    float h2_net = ((curr_left * h2.w1) + (curr_right * h2.w2) + h2.bias);
    float h3_net = ((curr_left * h3.w1) + (curr_right * h3.w2) + h3.bias);

    // Calculate hidden layer outputs
    float h1_out = sigmoid(h1_net);
    float h2_out = sigmoid(h2_net);
    float h3_out = sigmoid(h3_net);

    // Calculate output layer activations
    float o1_net = ((h1_out * o1.w1) + (h2_out * o1.w2) + (h3_out * o1.w3) + o1.bias);
    float o2_net = ((h1_out * o2.w1) + (h2_out * o2.w2) + (h3_out * o2.w3) + o2.bias);

    // Calculate output layer outputs
    float o1_out = sigmoid(o1_net);
    float o2_out = sigmoid(o2_net);

    // Convert to motor commands (0-255 range)
    computed_nodes.left_s = (uint8_t)(o1_out * 255);
    computed_nodes.right_s = (uint8_t)(o2_out * 255);

    return computed_nodes;
}


// UGLY MATH STUFF
void train_neural_network(motor_command target, motor_command out, uint8_t left_sensor, uint8_t right_sensor) {

}

void training() {
    // Check if data is available
    if (training_data == NULL) {
        clear_screen();
        lcd_cursor(0, 0);
        print_string("No data!");
        _delay_ms(2000);
        current_state = STATE_PROPORTIONAL;
        return;
    }
    
    // Display state
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training");
    _delay_ms(1000); 



    // Training complete
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training ");
    lcd_cursor(0, 1);
    print_string("Done");
    _delay_ms(1000); 

    while (!check_button()) { _delay_ms(100); }

    current_state = STATE_NEURAL_NETWORK;
}

void neural_network() {
    // Display state
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Neural");
    lcd_cursor(0, 1); 
    print_string("Network");
    _delay_ms(1000); 
    clear_screen();

}

void proportional(){
    // Display state
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Proportional");
    _delay_ms(1000); 



    // Run proportional control logic
    while (current_state == STATE_PROPORTIONAL) {
        
        // Check for button press to switch state
        if (check_button()) {
            current_state = STATE_DATA_COLLECTION;
            stop();
            break;
        }
        
        _delay_ms(5); // Control rate
    }

}

void data_collection() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Data Collection");
    _delay_ms(1000);
    clear_screen();
    
    int curr_left = 0;
    int curr_right = 0;
    
    // Reset sample counter
    sample_count = 0;
    
    // Collect sensor readings
    while (sample_count < MAX_INPUTS / 2) { // Assuming MAX_INPUTS is the max array size
        curr_left = analog(3);  // LEFT_EYE
        curr_right = analog(4); // RIGHT_EYE
        
        // Store directly in the global array
        sensor_val[sample_count].left = curr_left;
        sensor_val[sample_count].right = curr_right;
        
        // Display current readings
        clear_screen();
        lcd_cursor(0, 0);
        print_string("Data:");
        print_num(sample_count);
        print_string("    ");
        lcd_cursor(0, 1);
        print_string("L");
        print_num(curr_left);
        lcd_cursor(4, 1);
        print_string("R");
        print_num(curr_right);
        
        _delay_ms(70);
        
        sample_count++;
        
        // Check for button press to abort
        if (check_button()) {
            clear_screen();
            print_string("Collection aborted");
            _delay_ms(1000);
            clear_screen();
            break;
        }
    }
    
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Collected");
    lcd_cursor(0, 1);
    print_num(sample_count);
    print_string(" samples");
    _delay_ms(1000);
    clear_screen();
    
    // Move to training state
    current_state = STATE_TRAINING;
}

int main(void) {
    init();
    init_servo();
    init_lcd();
    stop(); 

    while(1){
        switch(current_state){
            case STATE_PROPORTIONAL:
                init_neural_network(); 
                proportional(); 
                break; 

            case STATE_DATA_COLLECTION:
                data_collection(); 
                break; 

            case STATE_TRAINING:
                training(); 
                break; 

            case STATE_NEURAL_NETWORK:	
                neural_network(); 
                break; 

            default:
                current_state = STATE_PROPORTIONAL; 
                break; 
        }	
    }
        
}