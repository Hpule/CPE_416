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

 // might not need this struct due to memory restrictions (board has 4 kB RAM)
 typedef struct motor_command { // struct for the motor command (left and right speed) 
	uint8_t left_s; // left sensor
	uint8_t right_s; // right sensor 
 } motor_command;

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




 // maybe make the weights and biases global variables so that they retain their values across functions 

 
 void stop() {
	set_servo(2, 127); // stop the left motor
	set_servo(3, 127); // stop the right motor
 } 

 float sigmoid(float x) { // squeezing function for a 0-1 output 
	float result = 1 / (1 + exp(-x)); // sigmoid activation function
	return result; 
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

 motor_command compute_neural_network(uint8_t left, uint8_t right) {
	motor_command neural_output; 
    
	
	
	return neural_output; // return the left and right motor commands
 }

 float calculate_wb(uint8_t target, float output, float hidden_out, float old_wb) { // calculate the weights and biases (wb stands for weights/biases)
	float do_dnet = output * (1 - output); 
	if () { // this is for if the neuron is a hidden layer neuron (different types of computation for de_do)
		float de_dout1 = hidden_out * do_dnet; // might be only one part of the equation and we need the other part (regarding do_dnet) 
		float de_dout2 = hidden_out2 * do_dnet; // might be only one part of the equation and we need the other part (regarding do_dnet) WRONG CODE 
		float det_dout = de_dout1 + det dout2; // the derivative of the output with respect to the net input 
		float de_do = (target - output); // the components of the output equation 
	} 

	else { // this is for if the neuron is an output layer neuron
		float de_do = (target - output); // the components of the output equation 
	}
	float dnet_dwb = hidden_out;
	float de_dwb = de_do * do_dnet * dnet_dwb; // the derivative of the output with respect to the weight

	float alpha = 0.3; // the learning rate (0.001 - 0.3, should decrease each time)
	
	float new_wb = old_wb - alpha * de_dwb; 

	return new_wb; // output the new weight/bias
 }

 char* collect_data() { // need to test if this works or not
	stop();
	char *command_arr = malloc(sizeof(uint8_t) * MAX_INPUTS); // allocate memory for the array of motor commands
	int i = 0; // index of the array 

	while(i <= MAX_INPUTS - 1) { // collect data from sensors until the array is full 
		uint8_t left_sensor = analog(3); // get the left and right sensor values
		uint8_t right_sensor = analog(4); 

		command_arr[i] = left_sensor; // put the commands into the array 
		command_arr[i+1] = right_sensor; // put the commands into the array
		i++; // increment the index of the array 
	}
	return command_arr; 
 }

 void train_neural_network(char *command_arr) { // from the data array we have the input values 
	stop();  
	int even = 0; // indicies for the left and right sensor values (even = left, right = odd)
	int odd = 1;
	
	// calculate the outputs of the hidden neurons (these values are the same for both output neurons)
	float hidden_out1 = sigmoid(command_arr[even] * h1.w1 + command_arr[odd] * h1.w2 + h1.bias); 
	float hidden_out2 = sigmoid(command_arr[even] * h2.w1 + command_arr[odd] * h2.w2 + h2.bias);
	float hidden_out3 = sigmoid(command_arr[even] * h3.w1 + command_arr[odd] * h3.w2 + h3.bias); 


	// calculate the outputs of the output neurons 
	float output1 = sigmoid(hidden_out1 * o1.w1 + hidden_out2 * o1.w2 + hidden_out3 * o1.w3 + o1.bias); 
	float output2 = sigmoid(hidden_out1 * o2.w1 + hidden_out2 * o2.w2 + hidden_out3 * o2.w3 + o2.bias); 
	
	// backpropagation 
	// might need to squeeze this with a sigmoid so that I'm not getting a value of 0-255
	motor_command target = compute_proportional(command_arr[even], command_arr[odd]); // get the motor commands from the neural network

	// start with the output layer (target is the output from the proportional - confimed by Dr.Seng) 
	// for first output layer neuron 
	float det_w7 = calculate_wb(target.left_s, output1, hidden_out1); 
	float det_w8 = calculate_wb(target.left_s, output1, hidden_out2); 
	float det_w9 = calculate_wb(target.left_s, output1, hidden_out3); 
	float det_b4 = calculate_wb(target.left_s, output1, 1); // bias is always 1 (for the output layer) 

	// for second output layer neuron 
	float det_w10 = calculate_wb(target.right_s, output2, hidden_out1);
	float det_w11 = calculate_wb(target.right_s, output2, hidden_out2);
	float det_w12 = calculate_wb(target.right_s, output2, hidden_out3);
	float det_b5 = calculate_wb(target.right_s, output2, 1); // bias is always 1 (for the output layer)

	
	// calculate the hidden layer weights and biases
	// for first hidden layer neuron 
	float det_w1 = calculate_wb(command_arr[even], hidden_out1, o1.w1);
	float det_w2 = calculate_wb(command_arr[odd], hidden_out1, o1.w2);
	float det_b1 = calculate_wb(command_arr[even], hidden_out1, o1.w3);

	// for second hidden layer neuron 
	float det_w3 = calculate_wb(command_arr[even], hidden_out2, o1.w1);
	float det_w4 = calculate_wb(command_arr[odd], hidden_out2, o1.w2);
	float det_b2 = calculate_wb(command_arr[even], hidden_out2, o1.w3);

	// for third hidden layer neuron
	float det_w5 = calculate_wb(command_arr[even], hidden_out3, o1.w1);
	float det_w6 = calculate_wb(command_arr[odd], hidden_out3, o1.w2);
	float det_b3 = calculate_wb(command_arr[even], hidden_out3, o1.w3);

	

	// probably take in an array of motor_commands and call compute_neural_network() for each one
	// then update the weights of the neural network after every epoch (may need a helper function for this)
 }
 
 void main(void) {


 }