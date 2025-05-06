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

// ---------------- Global Variables ----------------  
#define MAX_INPUTS 100 // max amount of sensor values (motor commands) into the  
#define HIDDEN_NEURONS 3 // number of hidden neurons in the neural network
#define LEARN_RATE 0.3

// All of the states of our code:
#define STATE_PROPORTIONAL 0
#define STATE_DATA_COLLECTION 1
#define STATE_TRAINING 2
#define STATE_NEURAL_NETWORK 3

#define KP 0.2  // Proportional gain
#define KI 0.01   // Integral gain

#define LEFT_MOTOR 2
#define LEFT_SENSOR 2
#define RIGHT_MOTOR 3
#define RIGHT_SENSOR 3
 
typedef struct nnv { // struct for the neural network variables (nnv)
    float w1; // initialize the weights and biases to random values (type cast b/c rand() returns int)
    float w2; 
    float w3; // this is just for the output layer since it has 3 inputs (from the hidden layer)
    float bias; 
}	nnv; 

// might not need this struct due to memory restrictions (board has 4 kB RAM)
typedef struct motor_command { // struct for the motor command (left and right speed) 
    uint8_t left_s; // left sensor
    uint8_t right_s; // right sensor 

} motor_command;

float learning_rate = 0.3; // start off high
uint8_t current_state = STATE_PROPORTIONAL; 
uint8_t training_data[200]; 


// neurons in the neural network (both hidden and output layers) - might not be wise to create instances of them like this, maybe just store all the values in an array
nnv h1; // first hidden neuron 
nnv h2; // second hidden neuron  
nnv h3; // third hidden neuron 
nnv o1; // first output neuron  
nnv o2; // second output neuron

// ---------------- Helper Functions ----------------  
nnv nnv_init(void) { 
    nnv neuron;
    neuron.w1 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.w2 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.w3 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.bias = (float)rand() / RAND_MAX; // random bias between 0 and 1

    return neuron; 
}

void stop() {
    set_servo(LEFT_MOTOR, 127); // stop the left motor
    set_servo(RIGHT_MOTOR, 127); // stop the right motor
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

void motor(struct motor_command speed_control) {
    uint8_t drive_left = ((60*(speed_control.left_s + 100))/(200))-30;
    uint8_t drive_right = ((60*(speed_control.right_s + 100))/(200))-30;

    set_servo(0, 127 + drive_left);
    set_servo(1, 127 - drive_right);
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

// ---------------- Main Functions ----------------  

motor_command compute_proportional(uint8_t left, uint8_t right) {
    motor_command both_commands; // initialize a motor_command struct
    float kp = 0.5; // proportional gain
    float left_command = kp * (left) + 127; // equation with only proportional 
    float right_command = kp * (right) + 127; // equation with only proportional

    both_commands.left_s = left_command; // assign the left command to the struct
    both_commands.right_s = right_command; // assign the right command to the struct

    return both_commands; // return the left and right command
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

void collect_data() {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Data");
    lcd_cursor(0,1); 
    print_string("Collection");
    _delay_ms(1000); 
    clear_screen();
    
    stop();
    int i = 0; // index of the array 

    while(i < 200) { // collect data from sensors until the array is full 
        uint8_t left_sensor = analog(LEFT_SENSOR); // get the left and right sensor values
        uint8_t right_sensor = analog(RIGHT_SENSOR); 

        training_data[i] = left_sensor; // put the commands into the array 
        training_data[i+1] = right_sensor; // put the commands into the array
        
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
    stop(); 
}

void proportional(){
    // Display state
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Proportional");
    _delay_ms(1000); 
    
    while (current_state == STATE_PROPORTIONAL) {
        // Read sensors
        uint8_t left_sensor = analog(LEFT_SENSOR);
        uint8_t right_sensor = analog(RIGHT_SENSOR);
        
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
        motor_command commands = compute_proportional(left_sensor, right_sensor);
        
        // Apply motor commands
        set_servo(LEFT_MOTOR, commands.left_s);
        set_servo(RIGHT_MOTOR, commands.right_s);
        
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
    collect_data(); 
    current_state = STATE_TRAINING; 
}

motor_command train_neural_network(uint8_t left_sval, uint8_t right_sval, nnv *h1, nnv *h2, nnv *h3, nnv *o1, nnv *o2) {
    motor_command target = compute_proportional(left_sval, right_sval); // get the motor commands from the neural network
    motor_command nn_output; 
    
    // feed forward algorithm 
    float net_h1 = h1->w1*left_sval + h1->w2*right_sval + h1->bias;
    float out_h1 = sigmoid(net_h1); // activation function for the hidden neuron

    float net_h2 = h2->w1*left_sval + h2->w2*right_sval + h2->bias;
    float out_h2 = sigmoid(net_h2); // activation function for the hidden neuron 

    float net_h3 = h3->w1*left_sval + h3->w2*right_sval + h3->bias;
    float out_h3 = sigmoid(net_h3); // activation function for the hidden neuron 

    float net_o1 = o1->w1*out_h1 + o1->w2*net_h2 + o1->w3*net_h3 + o1->bias;
    float out_o1 = sigmoid(net_o1); // activation function for the output neuron 

    float net_o2 = o2->w1*out_h1 + o2->w2*net_h2 + o2->w3*net_h3 + o2->bias;
    float out_o2 = sigmoid(net_o2); // activation function for the output neuron 

    // backpropagation algorithm
    
    // output neuron calculation 
    float det_dnet1 = (out_o1 - target.left_s) * (out_o1 * (1 - out_o1));
    float det_dw7 = det_dnet1 * out_h1; // the derivative of the output with respect to the net input
    float det_dw8 = det_dnet1 * out_h2; // the derivative of the output with respect to the net input
    float det_dw9 = det_dnet1 * out_h3; // the derivative of the output with respect to the net input
    float det_db4 = det_dnet1 * 1; // the derivative of the output with respect to the net input

    float det_dnet2 = (out_o2 - target.right_s) * (out_o2 * (1 - out_o2));
    float det_dw10 = det_dnet2 * out_h1; // the derivative of the output with respect to the net input
    float det_dw11 = det_dnet2 * out_h2; // the derivative of the output with respect to the net input
    float det_dw12 = det_dnet2 * out_h3; // the derivative of the output with respect to the net input
    float det_db5 = det_dnet2 * 1; // the derivative of the output with respect to the net input

    //Weight 1
    float de1_dout1 = det_dnet1 * o1->w1;  // math might be wrong again, I'm kind of getting lost in the sauce
    float de2_dout1 = det_dnet2 * o2->w1;
    float det_dout1 = de1_dout1 + de2_dout1; 
    float det_dw1 = det_dout1 * (out_h1 * (1 - out_h1)) * net_h1; // the derivative of the output with respect to the net input

    //Weight 2
     de1_dout1 = det_dnet1 * o1->w1;
     de2_dout1 = det_dnet2 * o2->w1;
     det_dout1 = de1_dout1 + de2_dout1;
    float det_dw2 = det_dout1 * (out_h1 * (1 - out_h1)) * net_h2; // the derivative of the output with respect to the net input

    // Bias 1
     de1_dout1 = det_dnet1 * o1->w1; 
     de2_dout1 = det_dnet2 * o2->w1;
     det_dout1 = de1_dout1 + de2_dout1; 
    float det_db1 = det_dout1 * (out_h1 * (1 - out_h1)) * 1; // the derivative of the output with respect to the net input

    // Weight 4
    float de1_dout2 = det_dnet1 * o1->w2;
    float de2_dout2 = det_dnet2 * o2->w2; 
    float det_dout2 = de1_dout2 + de2_dout2;
    float det_dw3 = det_dout2 * (out_h2 * (1 - out_h2)) * net_h1; // the derivative of the output with respect to the net input

     de1_dout2 = det_dnet1 * o1->w2;
     de2_dout2 = det_dnet2 * o2->w2;
     det_dout2 = de1_dout2 + de2_dout2;
    float det_dw4 = det_dout2 * (out_h2 * (1 - out_h2)) * net_h2; // the derivative of the output with respect to the net input
    
     de1_dout2 = det_dnet1 * o1->w2;
     de2_dout2 = det_dnet2 * o2->w2;
     det_dout2 = de1_dout2 + de2_dout2;
    float det_db2 = det_dout2 * (out_h2 * (1 - out_h2)) * 1; // the derivative of the output with respect to the net input
    
    float de1_dout3 = det_dnet1 * o1->w3;
    float de2_dout3 = det_dnet2 * o2->w3;
    float det_dout3 = de1_dout3 + de2_dout3;
    float det_dw5 = det_dout3 * (out_h3 * (1 - out_h3)) * net_h1; // the derivative of the output with respect to the net input
    
     de1_dout3 = det_dnet1 * o1->w3;
     de2_dout3 = det_dnet2 * o2->w3;
     det_dout3 = de1_dout3 + de2_dout3;
    float det_dw6 = det_dout3 * (out_h3 * (1 - out_h3)) * net_h2; // the derivative of the output with respect to the net input
    
     de1_dout3 = det_dnet1 * o1->w3;
     de2_dout3 = det_dnet2 * o2->w3;
     det_dout3 = de1_dout3 + de2_dout3;
    float det_db3 = det_dout3 * (out_h3 * (1 - out_h3)) * 1; // the derivative of the output with respect to the net input
    
    // update the weights and biases
    o1->w1 -= det_dw7 * learning_rate; // output layer first neuron 
    o1->w2 -= det_dw8 * learning_rate; 
    o1->w3 -= det_dw9 * learning_rate; 
    o1->bias -= det_db4 * learning_rate; 

    o2->w1 -= det_dw10 * learning_rate; // output layer second neuron
    o2->w2 -= det_dw11 * learning_rate; 
    o2->w3 -= det_dw12 * learning_rate; 
    o2->bias -= det_db5 * learning_rate; 
    
    h1->w1 -= det_dw1 * learning_rate; // hidden layer first neuron 
    h1->w2 -= det_dw2 * learning_rate; 
    h1->bias -= det_db1 * learning_rate; 
    
    h2->w1 -= det_dw3 * learning_rate; // hidden layer second neuron
    h2->w2 -= det_dw4 * learning_rate; 
    h2->bias -= det_db2 * learning_rate; 
    
    h3->w1 -= det_dw5 * learning_rate; // hidden layer third neuron
    h3->w2 -= det_dw6 * learning_rate; 
    h3->bias -= det_db3 * learning_rate; 

    nn_output.left_s = out_o1;
    nn_output.right_s = out_o2; 
    
    return nn_output; 
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
    stop(); 

    // Initialize neurons ONLY if it's the first time training
    static bool first_training = true;
    if (first_training) {
        h1 = nnv_init();
        h2 = nnv_init();
        h3 = nnv_init();
        o1 = nnv_init();
        o2 = nnv_init();
        first_training = false;
    }
    
    motor_command nn_out; 
    motor_command denormalized_output;

    for (int epochs = 0; epochs < MAX_INPUTS; epochs++) {
        for (int vals = 0; vals < 200; vals += 2) { // Iterate through all 100 pairs
            uint8_t left_sval = training_data[vals]; 
            uint8_t right_sval = training_data[vals+1]; 
            
            nn_out = train_neural_network(left_sval, right_sval, &h1, &h2, &h3, &o1, &o2);// train the neural network with the command array 
            learning_rate = learning_rate - 0.05;        
        }

        clear_screen();
        lcd_cursor(0, 0);
        print_string("Epochs");
        lcd_cursor(0,1);
        print_num(epochs);  
        _delay_ms(100); 
    }
    
    denormalized_output.left_s = denormalize(nn_out.left_s);
    denormalized_output.right_s = denormalize(nn_out.right_s);
    motor(denormalized_output);

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

motor_command compute_neural_network(uint8_t left_sval, uint8_t right_sval, nnv *h1, nnv *h2, nnv *h3, nnv *o1, nnv *o2){
    motor_command nn_output; 

    // feed forward algorithm 
    float net_h1 = h1->w1*left_sval + h1->w2*right_sval + h1->bias;
    float out_h1 = sigmoid(net_h1); // activation function for the hidden neuron

    float net_h2 = h2->w1*left_sval + h2->w2*right_sval + h2->bias;
    float out_h2 = sigmoid(net_h2); // activation function for the hidden neuron 

    float net_h3 = h3->w1*left_sval + h3->w2*right_sval + h3->bias;
    float out_h3 = sigmoid(net_h3); // activation function for the hidden neuron 

    float net_o1 = o1->w1*out_h1 + o1->w2*out_h2 + o1->w3*out_h3 + o1->bias;
    float out_o1 = sigmoid(net_o1); // activation function for the output neuron 

    float net_o2 = o2->w1*out_h1 + o2->w2*out_h2 + o2->w3*out_h3 + o2->bias;
    float out_o2 = sigmoid(net_o2); // activation function for the output neuron 
    
    nn_output.left_s = out_o1; 
    nn_output.right_s = out_o2; 

    return nn_output; 
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
        uint8_t left_sensor = analog(LEFT_SENSOR);
        uint8_t right_sensor = analog(RIGHT_SENSOR);
        
        // Display sensor values
        clear_screen();
        lcd_cursor(0, 0);
        print_string("L:");
        print_num(left_sensor);
        lcd_cursor(8, 0);
        print_string("R:");
        print_num(right_sensor);

        // Assuming you have a function to compute neural network output without training
        motor_command nn_out = compute_neural_network(left_sensor, right_sensor, &h1, &h2, &h3, &o1, &o2);
        
        // Denormalize and apply to motors
        motor_command denormalized_output;
        denormalized_output.left_s = denormalize(nn_out.left_s);
        denormalized_output.right_s = denormalize(nn_out.right_s);
        motor(denormalized_output);
        
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