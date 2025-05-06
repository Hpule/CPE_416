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
 
 // -------------------- Global Variables --------------------
 
 #define MAX_INPUTS 100 // max amount of sensor values (motor commands) into the  
 #define MAX_EPOCHS 50  // max amount of epochs for training
 
 // Learning rate - starts higher and decreases during training
 float learning_rate = 0.01;
 
 #define STATE_PROPORTIONAL 0
 #define STATE_DATA_COLLECTION 1
 #define STATE_TRAINING 2
 #define STATE_NEURAL_NETWORK 3
 
 #define KP 0.2  // Proportional gain
 #define KI 0.01 // Integral gain
 #define KD 0.05 // Derivative gain
 
 #define LEFT_MOTOR   0
 #define LEFT_SENSOR  0
 #define RIGHT_MOTOR  1
 #define RIGHT_SENSOR 1
 
 uint8_t current_state = STATE_PROPORTIONAL; 
 
 typedef struct motor_command { // struct for the motor command (left and right speed) 
     uint8_t left_s, right_s;
 } motor_command;
 
 typedef struct sensor_reading {
     uint8_t left, right;
 } sensor_reading;
 
 sensor_reading sensor_val[MAX_INPUTS];
 int sample_count = 0; 
 
 typedef struct hidden_neuron { // struct for hidden neurons
     float w1;   // Weight for input 1 (left sensor)
     float w2;   // Weight for input 2 (right sensor)
     float bias; // Bias term
     float output; // Output of the neuron
 } hidden_neuron;
 
 typedef struct output_neuron { // struct for output neurons
     float w1;   // Weight for hidden neuron 1
     float w2;   // Weight for hidden neuron 2
     float w3;   // Weight for hidden neuron 3
     float bias; // Bias term
     float output; // Output of the neuron
 } output_neuron;
 
 // Neural network neurons
 hidden_neuron h1, h2, h3;
 output_neuron o1, o2;
 
 // Function prototypes
 void init_neural_network(void);
 void stop(void);
 float sigmoid(float x);
 bool check_button(void);
 float normalize(uint8_t value);
 float denormalize(float value);
 uint16_t adjust_training_iterations(void);
 void data_collection(void);
 void proportional(void);
 motor_command compute_proportional(uint8_t left, uint8_t right);
 motor_command compute_neural_network(uint8_t sensor_left, uint8_t sensor_right);
 void train_neural_network(void);
 void update_network_weights(uint8_t left_val, uint8_t right_val, motor_command target);
 void neural_network(void);
 
 // -------------------- Helper Functions --------------------
 
 void init_neural_network(void) {
     // Seed random for weight initialization
     srand(analog(BATTERY_PIN)); // Use battery reading as a random seed
     
     // Initialize hidden neurons with random weights (-0.5 to 0.5)
     h1.w1 = ((float)rand() / RAND_MAX) - 0.5;
     h1.w2 = ((float)rand() / RAND_MAX) - 0.5;
     h1.bias = ((float)rand() / RAND_MAX) - 0.5;
     h1.output = 0;
     
     h2.w1 = ((float)rand() / RAND_MAX) - 0.5;
     h2.w2 = ((float)rand() / RAND_MAX) - 0.5;
     h2.bias = ((float)rand() / RAND_MAX) - 0.5;
     h2.output = 0;
     
     h3.w1 = ((float)rand() / RAND_MAX) - 0.5;
     h3.w2 = ((float)rand() / RAND_MAX) - 0.5;
     h3.bias = ((float)rand() / RAND_MAX) - 0.5;
     h3.output = 0;
     
     // Initialize output neurons with random weights (-0.5 to 0.5)
     o1.w1 = ((float)rand() / RAND_MAX) - 0.5;
     o1.w2 = ((float)rand() / RAND_MAX) - 0.5;
     o1.w3 = ((float)rand() / RAND_MAX) - 0.5;
     o1.bias = ((float)rand() / RAND_MAX) - 0.5;
     o1.output = 0;
     
     o2.w1 = ((float)rand() / RAND_MAX) - 0.5;
     o2.w2 = ((float)rand() / RAND_MAX) - 0.5;
     o2.w3 = ((float)rand() / RAND_MAX) - 0.5;
     o2.bias = ((float)rand() / RAND_MAX) - 0.5;
     o2.output = 0;
     
     // Display initialization message
     clear_screen();
     lcd_cursor(0, 0);
     print_string("Network Init");
     lcd_cursor(0, 1);
     print_string("Complete");
     _delay_ms(1000);
 }
 
 void stop(void) {
     set_servo(LEFT_MOTOR, 127);  // stop the left motor
     set_servo(RIGHT_MOTOR, 127); // stop the right motor
 } 
 
 float sigmoid(float x) { // squeezing function for a 0-1 output 
     return 1.0 / (1.0 + exp(-x)); // sigmoid activation function
 }
 
 // Function to handle button press (with debouncing)
 bool check_button(void) {
     if (get_btn()) {
         _delay_ms(200); // Debounce
         while (get_btn()) {
             _delay_ms(10); // Wait for button release
         }
         return true;
     }
     return false;
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
  
 uint16_t adjust_training_iterations(void) {
     uint16_t iterations = 10; // Default starting value
     
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
 
 // -------------------- Main Functions --------------------
 
 void data_collection(void) {
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
         curr_left = analog(LEFT_SENSOR); 
         curr_right = analog(RIGHT_SENSOR); 
         
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
 
 void proportional(void) {
     // Display state
     clear_screen();
     lcd_cursor(0, 0);
     print_string("Proportional");
     _delay_ms(1000); 
 
 
 
     // Run proportional control logic
     while (current_state == STATE_PROPORTIONAL) {
         // Read sensor values
         uint8_t left_sensor = analog(LEFT_SENSOR);
         uint8_t right_sensor = analog(RIGHT_SENSOR);
         
         // Display sensor values
         lcd_cursor(0, 0);
         print_string("L:");
         print_num(left_sensor);
         lcd_cursor(8, 0);
         print_string("R:");
         print_num(right_sensor);
         
         // Compute proportional control outputs
         motor_command cmd = compute_proportional(left_sensor, right_sensor);
         
         // Set motor speeds
         set_servo(LEFT_MOTOR, cmd.left_s);
         set_servo(RIGHT_MOTOR, cmd.right_s);
         
         // Check for button press to switch state
         if (check_button()) {
             current_state = STATE_DATA_COLLECTION;
             stop();
             break;
         }
         
         _delay_ms(5); // Control rate
     }
 }
 
 motor_command compute_proportional(uint8_t left, uint8_t right) {
    motor_command both_commands; // initialize a motor_command struct
    float kp = 0.5; // proportional gain
    float left_command = kp * (left) + 127; // equation with only proportional 
    float right_command = kp * (right) + 127; // equation with only proportional
    both_commands.left_speed = left_command; // assign the left command to the struct
    both_commands.right_speed = right_command; // assign the right command to the struct

    return both_commands; // return the left and right command
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
 
 void train_neural_network(void) {
     // Check if data is available
     if (sample_count == 0) {
         clear_screen();
         lcd_cursor(0, 0);
         print_string("No data!");
         _delay_ms(2000);
         current_state = STATE_PROPORTIONAL;
         return;
     }
     
     // Get number of training epochs
     uint16_t epochs = adjust_training_iterations();
     
     // Display training state
     clear_screen();
     lcd_cursor(0, 0);
     print_string("Training");
     _delay_ms(1000); 
     
     // For each epoch
     for (uint16_t epoch = 0; epoch < epochs; epoch++) {
         // Display progress
         lcd_cursor(0, 1);
         print_string("Epoch: ");
         print_num(epoch + 1);
         print_string("/");
         print_num(epochs);
         
         // For each training example
         for (int i = 0; i < sample_count; i++) {
             // Get sensor values
             uint8_t left_val = sensor_val[i].left;
             uint8_t right_val = sensor_val[i].right;
             
             // Get target outputs from proportional controller
             motor_command target = compute_proportional(left_val, right_val);
             
             // Update network weights
             update_network_weights(left_val, right_val, target);
         }
         
         // Gradually reduce learning rate
         learning_rate *= 0.95;
     }
     
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
 
 void update_network_weights(uint8_t left_val, uint8_t right_val, motor_command target) {
     // Normalize inputs and targets
     float norm_left = normalize(left_val);
     float norm_right = normalize(right_val);
     float target_left = normalize(target.left_s);
     float target_right = normalize(target.right_s);
     
     // Forward pass
     // Hidden layer
     float h1_net = (norm_left * h1.w1) + (norm_right * h1.w2) + h1.bias;
     h1.output = sigmoid(h1_net);
     
     float h2_net = (norm_left * h2.w1) + (norm_right * h2.w2) + h2.bias;
     h2.output = sigmoid(h2_net);
     
     float h3_net = (norm_left * h3.w1) + (norm_right * h3.w2) + h3.bias;
     h3.output = sigmoid(h3_net);
     
     // Output layer
     float o1_net = (h1.output * o1.w1) + (h2.output * o1.w2) + (h3.output * o1.w3) + o1.bias;
     o1.output = sigmoid(o1_net);
     
     float o2_net = (h1.output * o2.w1) + (h2.output * o2.w2) + (h3.output * o2.w3) + o2.bias;
     o2.output = sigmoid(o2_net);
     
     // Backpropagation
     // Output layer errors
     float o1_error = o1.output - target_left;
     float o2_error = o2.output - target_right;
     
     // Output layer gradients
     float o1_delta = o1_error * (o1.output * (1 - o1.output));
     float o2_delta = o2_error * (o2.output * (1 - o2.output));
     
     // Hidden layer errors
     float h1_error = (o1_delta * o1.w1) + (o2_delta * o2.w1);
     float h2_error = (o1_delta * o1.w2) + (o2_delta * o2.w2);
     float h3_error = (o1_delta * o1.w3) + (o2_delta * o2.w3);
     
     // Hidden layer gradients
     float h1_delta = h1_error * (h1.output * (1 - h1.output));
     float h2_delta = h2_error * (h2.output * (1 - h2.output));
     float h3_delta = h3_error * (h3.output * (1 - h3.output));
     
     // Update output layer weights
     o1.w1 -= learning_rate * o1_delta * h1.output;
     o1.w2 -= learning_rate * o1_delta * h2.output;
     o1.w3 -= learning_rate * o1_delta * h3.output;
     o1.bias -= learning_rate * o1_delta;
     
     o2.w1 -= learning_rate * o2_delta * h1.output;
     o2.w2 -= learning_rate * o2_delta * h2.output;
     o2.w3 -= learning_rate * o2_delta * h3.output;
     o2.bias -= learning_rate * o2_delta;
     
     // Update hidden layer weights
     h1.w1 -= learning_rate * h1_delta * norm_left;
     h1.w2 -= learning_rate * h1_delta * norm_right;
     h1.bias -= learning_rate * h1_delta;
     
     h2.w1 -= learning_rate * h2_delta * norm_left;
     h2.w2 -= learning_rate * h2_delta * norm_right;
     h2.bias -= learning_rate * h2_delta;
     
     h3.w1 -= learning_rate * h3_delta * norm_left;
     h3.w2 -= learning_rate * h3_delta * norm_right;
     h3.bias -= learning_rate * h3_delta;
 }
 
 void neural_network(void) {
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
         // Read sensor values
         uint8_t left_sensor = analog(LEFT_SENSOR);
         uint8_t right_sensor = analog(RIGHT_SENSOR);
         
         // Display sensor values
         lcd_cursor(0, 0);
         print_string("L:");
         print_num(left_sensor);
         lcd_cursor(8, 0);
         print_string("R:");
         print_num(right_sensor);
         
         // Compute neural network outputs
         motor_command cmd = compute_neural_network(left_sensor, right_sensor);
         
         // Set motor speeds
         set_servo(LEFT_MOTOR, cmd.left_s);
         set_servo(RIGHT_MOTOR, cmd.right_s);
         
         // Display motor values
         lcd_cursor(0, 1);
         print_string("ML:");
         print_num(cmd.left_s);
         lcd_cursor(8, 1);
         print_string("MR:");
         print_num(cmd.right_s);
         
         // Check for button press to switch state
         if (check_button()) {
             current_state = STATE_PROPORTIONAL;
             stop();
             break;
         }
         
         _delay_ms(50); // Control rate
     }
 }
 
 int main(void) {
     // Initialize hardware
     init();
     
     // Stop motors
     stop();
     
     // Initialize neural network
     init_neural_network();
     
     // Main state machine
     while(1) {
         switch(current_state) {
             case STATE_PROPORTIONAL:
                 proportional();
                 break;
                 
             case STATE_DATA_COLLECTION:
                 data_collection();
                 break;
                 
             case STATE_TRAINING:
                 train_neural_network();
                 break;
                 
             case STATE_NEURAL_NETWORK:
                 neural_network();
                 break;
                 
             default:
                 current_state = STATE_PROPORTIONAL;
                 break;
         }
     }
     
     return 0;
 }