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


// All of the states of our code:
#define STATE_PROPORTIONAL 0
#define STATE_DATA_COLLECTION 1
#define STATE_TRAINING 2
#define STATE_NEURAL_NETWORK 3

#define KP 0.2  // Proportional gain
#define KI 0.01   // Integral gain
#define KD 0.05  // Derivative gain
#define NUM_ERROR_SAMPLES 5  // Number of samples to keep for derivative

uint8_t current_state = STATE_PROPORTIONAL; 
char *training_data = NULL;

// might not need this struct due to memory restrictions (board has 4 kB RAM)
typedef struct motor_command { // struct for the motor command (left and right speed) 
    uint8_t left_s; // left sensor
    uint8_t right_s; // right sensor 

} motor_command;


typedef struct sensor_reading {
    uint8_t left;
    uint8_t right;
} sensor_reading;

sensor_reading sensor_val[MAX_INPUTS];

int sample_count = 0; 

typedef struct nnv { // struct for the neural network variables (nnv)
    float w1; // initialize the weights and biases to random values (type cast b/c rand() returns int)
    float w2; 
    float w3; // this is just for the output layer since it has 3 inputs (from the hidden layer)
    float bias; 
}	nnv; 

// neurons in the neural network (both hidden and output layers) - might not be wise to create instances of them like this, maybe just store all the values in an array
nnv h1; // first hidden neuron 
nnv h2; // second hidden neuron  
nnv h3; // third hidden neuron 
nnv o1; // first output neuron  
nnv o2; // second output neuron

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


// maybe make the weights and biases global variables so that they retain their values across functions 


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

motor_command compute_pid(uint8_t left, uint8_t right) {
    motor_command pid_command;
    static int error_samples[NUM_ERROR_SAMPLES] = {0};
    static int integral = 0;
    static int prev_error = 0;
    
    // Calculate current error (difference between sensors)
    // For line following, typically we want left - right
    int error = left - right;
    
    // Update error history for derivative calculation
    for (int i = NUM_ERROR_SAMPLES - 1; i > 0; i--) {
        error_samples[i] = error_samples[i - 1];
    }
    error_samples[0] = error;
    
    // Calculate derivative (change in error)
    int derivative = error - prev_error;
    
    // Calculate integral (sum of errors over time)
    integral += error;
    
    // Limit integral to prevent wind-up (use smaller values)
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
    // Calculate PID output
    int pid_output = KP * error + KI * integral + KD * derivative;
    
    // Calculate motor commands based on PID output
    // For differential drive, motors move in opposite directions
    int left_command = 127 + pid_output;
    int right_command = 127 - pid_output;
    
    // Ensure motor commands stay within valid range
    if (left_command < 0) left_command = 0;
    if (left_command > 255) left_command = 255;
    if (right_command < 0) right_command = 0;
    if (right_command > 255) right_command = 255;
    
    pid_command.left_s = left_command;
    pid_command.right_s = right_command;
    
    // Save current error for next derivative calculation
    prev_error = error;
    
    return pid_command;
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
        // Read accelerometer values
        uint8_t accel_y = get_accel_y();
        uint8_t accel_x = get_accel_x();
        
        // Tilted left - decrease by 1
        if (accel_y > 30 && accel_y < 70) {iterations--; _delay_ms(100);}
        
        // Tilted right - increase by 1
        if (accel_y > 200 && accel_y < 250) {iterations++; _delay_ms(100);}
        
        // Tilted down - decrease by 10
        if (accel_x > 200 && accel_x < 230) {
            if (iterations >= 10) {
                iterations -= 10;
            } else {
                iterations = 0;
            }
            _delay_ms(100);
        }
        
        // Tilted up - increase by 10
        if (accel_x > 20 && accel_x < 70) {
            iterations += 10;
            _delay_ms(100);
        }
        
        // Ensure iterations doesn't go negative
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
    
    // Confirmation screen
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training with:");
    lcd_cursor(0, 1);
    print_num(iterations);
    print_string(" iterations");
    _delay_ms(1000);
    
    return iterations;
}

motor_command compute_neural_network(uint8_t left, uint8_t right) {
    motor_command neural_output; 
    
    float hidden_out1 = sigmoid(left * h1.w1 + right * h1.w2 + h1.bias);
    float hidden_out2 = sigmoid(left * h2.w1 + right * h2.w2 + h2.bias); 
    float hidden_out3 = sigmoid(left * h3.w1 + right * h3.w2 + h3.bias);

    // Calculate output layer (corrected the syntax error)
    float out_left = sigmoid(hidden_out1 * o1.w1 + hidden_out2 * o1.w2 + hidden_out3 * o1.w3 + o1.bias); 
    float out_right = sigmoid(hidden_out1 * o2.w1 + hidden_out2 * o2.w2 + hidden_out3 * o2.w3 + o2.bias);

    neural_output.left_s = (uint8_t)(out_left * 255); 
    neural_output.right_s = (uint8_t)(out_right * 255); 
    
    return neural_output;
}

float calculate_wb(float target, float output, float hidden_out, float old_wb, bool is_hidden_neuron) {
    float do_dnet = output * (1 - output);
    float de_do = (target - output);
    float dnet_dwb = hidden_out;
    float de_dwb = de_do * do_dnet * dnet_dwb;
    
    float alpha = 0.3; // learning rate
    float new_wb = old_wb - alpha * de_dwb;
    
    return new_wb;
}

char* collect_data() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Data");
    lcd_cursor(0,1); 
    print_string("Collection");
    _delay_ms(1000); 
    clear_screen();
    
    stop();
    char *command_arr = malloc(sizeof(uint8_t) * MAX_INPUTS); // allocate memory for the array of motor commands
    int i = 0; // index of the array 

    while(i < MAX_INPUTS - 1) { // collect data from sensors until the array is full 
        uint8_t left_sensor = analog(3); // get the left and right sensor values
        uint8_t right_sensor = analog(4); 

        command_arr[i] = left_sensor; // put the commands into the array 
        command_arr[i+1] = right_sensor; // put the commands into the array
        
        char lll[8];
        char rrr[8];

        sprintf(lll, "L:%d %d", left_sensor, i/2); // Display sample count, not array index
        sprintf(rrr, "R:%d", right_sensor);

        clear_screen();
        lcd_cursor(0,0);
        print_string(lll); 
        lcd_cursor(0,1);
        print_string(rrr);
        _delay_ms(100);
        clear_screen();
        
        i += 2; // Increment by 2 to move to the next pair
    }

    clear_screen();
    lcd_cursor(0, 0);
    print_string("Collection");
    lcd_cursor(0,1); 
    print_string("Done");
    _delay_ms(1000); 
    clear_screen();

    return command_arr; 
}

void proportional(){
    // Display state
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Proportional");
    _delay_ms(1000); 

    
    // Run proportional control logic
    while (current_state == STATE_PROPORTIONAL) {
        // Read sensors
        uint8_t left_sensor = analog(3);
        uint8_t right_sensor = analog(4);
        
        // Display sensor values (optional, useful for debugging)
        char left_str[8], right_str[8];
        sprintf(left_str, "L:%d", left_sensor);
        sprintf(right_str, "R:%d", right_sensor);
        
        clear_screen();
        lcd_cursor(0, 0);
        print_string(left_str);
        lcd_cursor(0, 1);
        print_string(right_str);

        // Calculate motor commands
        motor_command commands = compute_pid(left_sensor, right_sensor);
        
        // Apply motor commands
        set_servo(2, commands.left_s);
        set_servo(3, commands.right_s);
        
        // Check for button press to switch state
        if (check_button()) {
            current_state = STATE_DATA_COLLECTION;
            stop();
            break;
        }
        
        _delay_ms(5); // Control rate
    }

}

void data_collection(){
    training_data = collect_data(); 
    current_state = STATE_TRAINING; 
}

// UGLY MATH STUFF
void train_neural_network(motor_command target, motor_command out, uint8_t left_sensor, uint8_t right_sensor) {
    float db_left = normalize(left_sensor);
    float db_right = normalize(right_sensor);

    // Calculate hidden layer outputs for backpropagation
    float h1_net = (db_left * h1.w1 + db_right * h1.w2) - h1.bias;
    float h2_net = (db_left * h2.w1 + db_right * h2.w2) - h2.bias;
    float h3_net = (db_left * h3.w1 + db_right * h3.w2) - h3.bias;
    
    float h1_out = sigmoid(h1_net);
    float h2_out = sigmoid(h2_net);
    float h3_out = sigmoid(h3_net);

    // Target and output should be normalized (0-1)
    float target_left_norm = (float)target.left_s / 255.0;
    float target_right_norm = (float)target.right_s / 255.0;
    float output_left = (float)out.left_s / 255.0;
    float output_right = (float)out.right_s / 255.0;
    
    //------ Hidden Layer Weight Updates -----
    // Hidden neuron 1
    float new_h1_w1 = h1.w1 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w1) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w1)) *
        (h1_out * (1 - h1_out)) * db_left));
        
    float new_h1_w2 = h1.w2 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w1) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w1)) *
        (h1_out * (1 - h1_out)) * db_right));
        
    float new_h1_bias = h1.bias - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w1) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w1)) *
        (h1_out * (1 - h1_out)) * -1));
    
    // Hidden neuron 2
    float new_h2_w1 = h2.w1 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w2) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w2)) *
        (h2_out * (1 - h2_out)) * db_left));
        
    float new_h2_w2 = h2.w2 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w2) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w2)) *
        (h2_out * (1 - h2_out)) * db_right));
        
    float new_h2_bias = h2.bias - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w2) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w2)) *
        (h2_out * (1 - h2_out)) * -1));
    
    // Hidden neuron 3
    float new_h3_w1 = h3.w1 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w3) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w3)) *
        (h3_out * (1 - h3_out)) * db_left));
        
    float new_h3_w2 = h3.w2 - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w3) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w3)) *
        (h3_out * (1 - h3_out)) * db_right));
        
    float new_h3_bias = h3.bias - (LEARN_RATE *
        ((((output_left - target_left_norm) * (output_left * (1 - output_left)) * o1.w3) +
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * o2.w3)) *
        (h3_out * (1 - h3_out)) * -1));
    
    //------ Output Layer Weight Updates -----
    // Output neuron 1 (left motor)
    float new_o1_w1 = o1.w1 - (LEARN_RATE *
        ((output_left - target_left_norm) * (output_left * (1 - output_left)) * (h1_out)));
    float new_o1_w2 = o1.w2 - (LEARN_RATE *
        ((output_left - target_left_norm) * (output_left * (1 - output_left)) * (h2_out)));
    float new_o1_w3 = o1.w3 - (LEARN_RATE *
        ((output_left - target_left_norm) * (output_left * (1 - output_left)) * (h3_out)));
    float new_o1_bias = o1.bias - (LEARN_RATE *
        ((output_left - target_left_norm) * (output_left * (1 - output_left)) * (-1)));
    
    // Output neuron 2 (right motor)
    float new_o2_w1 = o2.w1 - (LEARN_RATE *
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * (h1_out)));
    float new_o2_w2 = o2.w2 - (LEARN_RATE *
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * (h2_out)));
    float new_o2_w3 = o2.w3 - (LEARN_RATE *
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * (h3_out)));
    float new_o2_bias = o2.bias - (LEARN_RATE *
        ((output_right - target_right_norm) * (output_right * (1 - output_right)) * (-1)));
    
    // Update all weights and biases
    h1.w1 = new_h1_w1;
    h1.w2 = new_h1_w2;
    h1.bias = new_h1_bias;
    
    h2.w1 = new_h2_w1;
    h2.w2 = new_h2_w2;
    h2.bias = new_h2_bias;
    
    h3.w1 = new_h3_w1;
    h3.w2 = new_h3_w2;
    h3.bias = new_h3_bias;
    
    o1.w1 = new_o1_w1;
    o1.w2 = new_o1_w2;
    o1.w3 = new_o1_w3;
    o1.bias = new_o1_bias;
    
    o2.w1 = new_o2_w1;
    o2.w2 = new_o2_w2;
    o2.w3 = new_o2_w3;
    o2.bias = new_o2_bias;
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

    // Get training iterations from user
    uint16_t training_iterations = adjust_training_iterations();
    float total_error = 0.0;
    
    // Training loop
    for (uint16_t epoch = 0; epoch < training_iterations; epoch++) {
        total_error = 0.0;
        
        // Process all data points in pairs (left and right sensor values)
        for (int i = 0; i < MAX_INPUTS - 1; i += 2) {
            uint8_t left_sensor = training_data[i];
            uint8_t right_sensor = training_data[i+1];
            
            if (left_sensor == 0 && right_sensor == 0) {
                continue; // Skip empty entries
            }
            
            // Get target outputs from PID controller
            motor_command target = compute_pid(left_sensor, right_sensor);
            
            // Get current network outputs
            motor_command out = compute_neural_network(left_sensor, right_sensor);
            
            // Train the network
            train_neural_network(target, out, left_sensor, right_sensor);
            
            // Calculate error for display
            float target_left_norm = (float)target.left_s / 255.0;
            float target_right_norm = (float)target.right_s / 255.0;
            float output_left = (float)out.left_s / 255.0;
            float output_right = (float)out.right_s / 255.0;
            total_error += fabs(target_left_norm - output_left) + fabs(target_right_norm - output_right);
        }
        
        // Display progress every 5 epochs or on the final epoch
        if (epoch % 5 == 0 || epoch == training_iterations - 1) {
            clear_screen();
            lcd_cursor(0, 0);
            print_string("Epoch: ");
            print_num(epoch);
            
            lcd_cursor(0, 1);
            print_string("Err: ");
            // Display average error
            float avg_error = total_error / (MAX_INPUTS / 2);
            print_num((uint16_t)(avg_error * 100)); // Scale by 100 for better visibility
            
            _delay_ms(200);  // Delay for display update
        }
        
        // Check for button press to abort training
        if (check_button()) {
            clear_screen();
            lcd_cursor(0, 0);
            print_string("Training");
            lcd_cursor(0, 1);
            print_string("Aborted");
            _delay_ms(1000);
            break;
        }
    }
    
    // Training complete
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training ");
    lcd_cursor(0, 1);
    print_string("Done");
    _delay_ms(1000); 

    // Wait for button press to continue
    while (!check_button()) {
        _delay_ms(100);
    }

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

    // Run neural network control loop
    while (current_state == STATE_NEURAL_NETWORK) {
        // Read sensors
        uint8_t left_sensor = analog(3);
        uint8_t right_sensor = analog(4);
        
        // Display sensor values
        char left_str[8], right_str[8];
        sprintf(left_str, "L:%d", left_sensor);
        sprintf(right_str, "R:%d", right_sensor);
        
        clear_screen();
        lcd_cursor(0, 0);
        print_string(left_str);
        lcd_cursor(0, 1);
        print_string(right_str);

        // Get motor commands from neural network
        motor_command commands = compute_neural_network(left_sensor, right_sensor);
        
        // Apply motor commands
        set_servo(2, commands.left_s);
        set_servo(3, commands.right_s);
        
        // Check for button press to switch back to training
        if (check_button()) {
            clear_screen();
            lcd_cursor(0, 0);
            print_string("More");
            lcd_cursor(0, 1); 
            print_string("Training");
            _delay_ms(1000); 
            clear_screen();

            current_state = STATE_TRAINING;
            stop();
            return;
        }
        
        _delay_ms(50); // Control rate
    }
}

int main(void) {
    // init_neural_network(); 
    init();
    init_servo();
    init_lcd();
    stop(); 

    while(1){
        switch(current_state){
            case STATE_PROPORTIONAL:
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