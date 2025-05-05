#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define MAX_EPOCHS 1000 // max amount of epochs (iterations) for the training loop - an arbitrary number 

float learning_rate = 0.3; // start off high


char *command_arr[100] = {
    10, 20, 30, 40, 50, 60, 70, 80, 90, 100,
    110, 120, 130, 140, 150, 160, 170, 180, 190, 200,
}; 

typedef struct nnv {
    float w1; // initialize the weights and biases to random values (type cast b/c rand() returns int)
    float w2; 
    float w3; // this is just for the output layer since it has 3 inputs (from the hidden layer)
    float bias; 
} nnv;

typedef struct motor_command { // struct for the motor command (left and right speed) 
	uint8_t left_s; // left sensor
	uint8_t right_s; // right sensor 
} motor_command;


nnv nnv_init(void) { 
    nnv neuron;
    neuron.w1 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.w2 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.w3 = (float)rand() / RAND_MAX; // random weight between 0 and 1
    neuron.bias = (float)rand() / RAND_MAX; // random bias between 0 and 1

    return neuron; 
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


float activation_function(float x) { // squeezing function for a 0-1 output 
    float result = 1 / (1 + exp(-x)); // sigmoid activation function
    return result; 
}



void train_neural_network(uint8_t left_sval, uint8_t right_sval, nnv h1, nnv h2, nnv h3, nnv o1, nnv o2) {
    motor_command target = compute_proportional(left_sval, right_sval); // get the motor commands from the neural network
    
    // feed forward algorithm 
    float net_h1 = h1.w1*left_sval + h1.w2*right_sval + h1.bias; 
    float out_h1 = activation_function(net_h1); // activation function for the hidden neuron

    float net_h2 = h2.w1*left_sval + h2.w2*right_sval + h2.bias;
    float out_h2 = activation_function(net_h2); // activation function for the hidden neuron 

    float net_h3 = h3.w1*left_sval + h3.w2*right_sval + h3.bias;
    float out_h3 = activation_function(net_h3); // activation function for the hidden neuron 

    float net_o1 = o1.w1*net_h1 + o1.w2*net_h2 + o1.w3*net_h3 + o1.bias;
    float out_o1 = activation_function(net_o1); // activation function for the output neuron 

    float net_o2 = o2.w1*net_h1 + o2.w2*net_h2 + o2.w3*net_h3 + o2.bias;
    float out_o2 = activation_function(net_o2); // activation function for the output neuron 

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

    // hidden neuron calculation 
    float de1_dout1 = det_dnet1 * o1.w1;  // math might be wrong again, I'm kind of getting lost in the sauce
    float de2_dout1 = det_dnet2 * o2.w1;
    float det_dout1 = de1_dout1 + de2_dout1; 
    float det_dw1 = det_dout1 * (out_h1 * (1 - out_h1)) * net_h1; // the derivative of the output with respect to the net input

    float de1_dout1 = det_dnet1 * o1.w1;
    float de2_dout1 = det_dnet2 * o2.w1;
    float det_dout1 = de1_dout1 + de2_dout1;
    float det_dw2 = det_dout1 * (out_h1 * (1 - out_h1)) * net_h2; // the derivative of the output with respect to the net input

    float de1_dout1 = det_dnet1 * o1.w1; 
    float de2_dout1 = det_dnet2 * o2.w1;
    float det_dout1 = de1_dout1 + de2_dout1; 
    float det_db1 = det_dout1 * (out_h1 * (1 - out_h1)) * 1; // the derivative of the output with respect to the net input

    float de1_dout2 = det_dnet1 * o1.w2;
    float de2_dout2 = det_dnet2 * o2.w2; 
    float det_dout2 = de1_dout2 + de2_dout2;
    float det_dw3 = det_dout2 * (out_h2 * (1 - out_h2)) * net_h1; // the derivative of the output with respect to the net input

    float de1_dout2 = det_dnet1 * o1.w2;
    float de2_dout2 = det_dnet2 * o2.w2;
    float det_dout2 = de1_dout2 + de2_dout2;
    float det_dw4 = det_dout2 * (out_h2 * (1 - out_h2)) * net_h2; // the derivative of the output with respect to the net input
    
    float de1_dout2 = det_dnet1 * o1.w2;
    float de2_dout2 = det_dnet2 * o2.w2;
    float det_dout2 = de1_dout2 + de2_dout2;
    float det_db2 = det_dout2 * (out_h2 * (1 - out_h2)) * 1; // the derivative of the output with respect to the net input
    
    float de1_dout3 = det_dnet1 * o1.w3;
    float de2_dout3 = det_dnet2 * o2.w3;
    float det_dout3 = de1_dout3 + de2_dout3;
    float det_dw5 = det_dout3 * (out_h3 * (1 - out_h3)) * net_h1; // the derivative of the output with respect to the net input
    
    float de1_dout3 = det_dnet1 * o1.w3;
    float de2_dout3 = det_dnet2 * o2.w3;
    float det_dout3 = de1_dout3 + de2_dout3;
    float det_dw6 = det_dout3 * (out_h3 * (1 - out_h3)) * net_h2; // the derivative of the output with respect to the net input
    
    float de1_dout3 = det_dnet1 * o1.w3;
    float de2_dout3 = det_dnet2 * o2.w3;
    float det_dout3 = de1_dout3 + de2_dout3;
    float det_db3 = det_dout3 * (out_h3 * (1 - out_h3)) * 1; // the derivative of the output with respect to the net input
    
    // update the weights and biases
    o1.w1 -= det_dw7 * learning_rate; // output layer first neuron 
    o1.w2 -= det_dw8 * learning_rate; 
    o1.w3 -= det_dw9 * learning_rate; 
    o1.bias -= det_db4 * learning_rate; 

    o2.w1 -= det_dw10 * learning_rate; // output layer second neuron
    o2.w2 -= det_dw11 * learning_rate; 
    o2.w3 -= det_dw12 * learning_rate; 
    o2.bias -= det_db5 * learning_rate; 
    
    h1.w1 -= det_dw1 * learning_rate; // hidden layer first neuron 
    h1.w2 -= det_dw2 * learning_rate; 
    h1.bias -= det_db1 * learning_rate; 
    
    h2.w1 -= det_dw3 * learning_rate; // hidden layer second neuron
    h2.w2 -= det_dw4 * learning_rate; 
    h2.bias -= det_db2 * learning_rate; 
    
    h3.w1 -= det_dw5 * learning_rate; // hidden layer third neuron
    h3.w2 -= det_dw6 * learning_rate; 
    h3.bias -= det_db3 * learning_rate; 

}



int main(void) {
    nnv h1 = nnv_init(); // initialize the 3 hidden neurons in the hidden layer 
    nnv h2 = nnv_init();
    nnv h3 = nnv_init();
    nnv o1 = nnv_init(); // initialize the 2 output neurons in the output layer 
    nnv o2 = nnv_init();
    int epochs = 0; // number of epochs (iterations) for the training loop

    for (epochs; epochs < MAX_EPOCHS; epochs++) {
        for (int vals = 0; vals < 100; vals++) { // loop through the command array 
            uint8_t left_sval = command_arr[vals]; // get the left sensor value from the command array
            uint8_t right_sval = command_arr[vals]; // get the right sensor value from the command array
            train_neural_network(left_sval, right_sval, h1, h2, h3, o1, o2); // train the neural network with the command array 
            learning_rate = learning_rate - 0.05;
        }
    }
}