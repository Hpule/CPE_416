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
	float w1 = (float)rand()/RAND_MAX; // initialize the weights and biases to random values (type cast b/c rand() returns int)
	float w2 = (float)rand()/RAND_MAX; 
	float w3 = (float)rand()/RAND_MAX; // this is just for the output layer since it has 3 inputs (from the hidden layer)
	float bias = (float)rand()/RAND_MAX; 
 }	nnv; 

 // neurons in the neural network (both hidden and output layers) - might not be wise to create instances of them like this, maybe just store all the values in an array
 nnv h1; // first hidden neuron 
 nnv h2; // second hidden neuron   
 nnv h3; // third hidden neuron 
 nnv o1; // first output neuron  
 nnv o2; // second output neuron

 motor_command compute_proportional(uint8_t left, uint8_t right); // function prototype for PID controller (only proportional) 
 void compute_neural_network(); // takes 2 uint8_t sensor values and returns a struct containing 2 motor values
 void collect_data(); // collects data from the sensors and stores it in an array for use in training 
 void calculate_wb(); // calculates the weights and biases of the neural network 
 void train_neural_network(); // trains the neural network using actual values returned from the sensors (stored in an array from the data capture mode)
 void update_hidden_layer(); // updates the hidden layer of the neural network
 void update_output_layer(); // updates the output layer of the neural network
 void sigmoid(); // sigmoid activation function
 void stop(); // stops the motors 

 // maybe make the weights and biases global variables so that they retain their values across functions 

 
 void stop() {
	set_servo(2, 127); // stop the left motor
	set_servo(3, 127); // stop the right motor
 } 

 void sigmoid(float x) { // squeezing function for a 0-1 output 
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

 void calculate_wb() { // calculate the weights and biases 

 }

 char* collect_data() { // need to test if this works or not
	stop();
	char *command_arr = malloc(sizeof(uint8_t) * MAX_INPUTS); // allocate memory for the array of motor commands
	int i = 0; // index of the array 

	while(i <= MAX_INPUTS - 1) { // collect data from sensors until the array is full 
		uint8_t left_sensor = analog(3); // get the left and right sensor values
		uint8_t right_sensor = analog(4); 

		comand_arr[i] = left_sensor; // put the commands into the array 
		command_arr[i+1] = right_sensor; // put the commands into the array
		i++; // increment the index of the array 
	}
	return command_arr 
 }

 void train_neural_network(char *command_arr) { // from the data array we have the input values 
	stop();  
	int even = 0; // indicies for the left and right sensor values (even = left, right = odd)
	int odd = 1;
	
	// calculate the outputs of the hidden neurons 
	float hidden_out1 = sigmoid(command_arr[even] * h1.w1 + command_arr[odd] * h1.w2 + h1.bias); 
	float hidden_out2 = sigmoid(command_arr[even] * h2.w1 + command_arr[odd] * h2.w2 + h2.bias);
	float hidden_out3 = sigmoid(command_arr[even] * h3.w1 + command_arr[odd] * h3.w2 + h3.bias); 

	// calculate the outputs of the outout neurons 
	float output1 = sigmoid(hidden_out1 * o1.w1 + hidden_out2 * o1.w2 + hidden_out3 * o1.w3 + o1.bias); 
	float output2 = sigmoid(hidden_out1 * o2.w1 + hidden_out2 * o2.w2 + hidden_out3 * o2.w3 + o2.bias); 
	
	// backpropagation 
	

	motor_command target = compute_proportional(command_arr[even], command_arr[odd]); // get the motor commands from the neural network

	// start with the output layer (target is the output from the proportional - confimed by Dr.Seng) 
	float de_w5 = (target.left_s - output1); // error for the first output neuron
	float out_term2 = (target.right_s - output2); // error for the second output neuron



	

	

	// probably take in an array of motor_commands and call compute_neural_network() for each one
	// then update the weights of the neural network after every epoch (may need a helper function for this)
 }
 
 void main(void) {


 }