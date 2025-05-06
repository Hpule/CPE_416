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
#define LEARN_RATE 0.9
#define MAX_EPOCHS 1000 // max amount of epochs (iterations) for the training loop - an arbitrary number 

// All of the states of our code:
#define STATE_PROPORTIONAL 0
#define STATE_DATA_COLLECTION 1
#define STATE_TRAINING 2
#define STATE_NEURAL_NETWORK 3

#define KP 0.2  // Proportional gain
#define KI 0.01   // Integral gain
#define KD 0.05  // Derivative gain
#define NUM_ERROR_SAMPLES 5  // Number of samples to keep for derivative


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

int sample_count = 0; 
float learning_rate = 0.3; // start off high
uint8_t current_state = STATE_PROPORTIONAL; 
char *training_data = NULL;

uint8_t command_arr[100] = {
    10, 20, 30, 40, 50, 60, 70, 80, 90, 100,
    110, 120, 130, 140, 150, 160, 170, 180, 190, 200
};

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

motor_command compute_proportional(uint8_t left, uint8_t right) { // we will put the neural output from compute_neural_network() here
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
        motor_command commands = compute_proportional(left_sensor, right_sensor);
        
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

void train_neural_network(uint8_t left_sval, uint8_t right_sval, nnv *h1, nnv *h2, nnv *h3, nnv *o1, nnv *o2) {
    motor_command target = compute_proportional(left_sval, right_sval); // get the motor commands from the neural network
    
    // feed forward algorithm 
    float net_h1 = h1->w1*left_sval + h1->w2*right_sval + h1->bias;
    float out_h1 = sigmoid(net_h1); // activation function for the hidden neuron

    float net_h2 = h2->w1*left_sval + h2->w2*right_sval + h2->bias;
    float out_h2 = sigmoid(net_h2); // activation function for the hidden neuron 

    float net_h3 = h3->w1*left_sval + h3->w2*right_sval + h3->bias;
    float out_h3 = sigmoid(net_h3); // activation function for the hidden neuron 

    float net_o1 = o1->w1*net_h1 + o1->w2*net_h2 + o1->w3*net_h3 + o1->bias;
    float out_o1 = sigmoid(net_o1); // activation function for the output neuron 

    float net_o2 = o2->w1*net_h1 + o2->w2*net_h2 + o2->w3*net_h3 + o2->bias;
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

    nnv h1 = nnv_init(); // initialize the 3 hidden neurons in the hidden layer 
    nnv h2 = nnv_init();
    nnv h3 = nnv_init();
    nnv o1 = nnv_init(); // initialize the 2 output neurons in the output layer 
    nnv o2 = nnv_init();

    for (int epochs = 0; epochs < MAX_EPOCHS; epochs++) {
        for (int vals = 0; vals < 100; vals++) { // loop through the command array 
            uint8_t left_sval = command_arr[vals]; // get the left sensor value from the command array
            uint8_t right_sval = command_arr[vals]; // get the right sensor value from the command array
            train_neural_network(left_sval, right_sval, &h1, &h2, &h3, &o1, &o2);// train the neural network with the command array 
            learning_rate = learning_rate - 0.05;
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



struct motor_command compute_neural_network(uint8_t left, uint8_t right){

    struct motor_command speed_control;
   
    double norm_left = left/255.0;
    double norm_right = right/255.0;
    
    h1.input_l = norm_left;
    h1.input_r = norm_right;
    h2.input_l = norm_left;
    h2.input_r = norm_right;
    h3.input_l = norm_left;
    h3.input_r = norm_right;
    
    //calculate the outputs of hidden layer
    compute_neuron_h(&h1);
    compute_neuron_h(&h2);
    compute_neuron_h(&h3);
        
    o1.input_1 = h1.output;
    o1.input_2 = h2.output;
    o1.input_3 = h3.output;
    o2.input_1 = h1.output;
    o2.input_2 = h2.output;
    o2.input_3 = h3.output;
    
//    h1.o1_wt = o1.weight_1;
//    h1.o2_wt = o2.weight_1;
//    h2.o1_wt = o1.weight_2;
//    h2.o2_wt = o2.weight_2;
//    h3.o1_wt = o1.weight_3;
//    h3.o2_wt = o2.weight_3;
    
    //calculate the outputs of the output layer
    compute_neuron_o(&o1);
    compute_neuron_o(&o2);
        
    //set the speed struct
    
    speed_control.left_speed = (int8_t)((200.0 * o1.output)-100.0);
    speed_control.right_speed = (int8_t)((200.0 * o2.output)-100.0);
    lcd_cursor(0,1);
    print_num((int8_t)((200.0 * o1.output)-100.0));
    lcd_cursor(5,1);
    print_num((int8_t)((200.0 * o2.output)-100.0));
    
    return speed_control;
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