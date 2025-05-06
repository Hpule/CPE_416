
/* ----------------------------------------------------------------------------
 *  Jonathon Hildreth and Eva Robin
 *  Assignment: Lab 3
 *  Description: Neural Net line following robot thing. Starts in proportional,
 *               and will follow lines. press button to collect 100 pieces of
 *               data, then press again and tilt to set number of training
 *               epochs. From here, pressing will switch between training one
 *               entire run-through of all epochs, and running the neural net. 
 *              
 *               Don't sue us if you train this thing too much and it turns 
 *               into Skynet.
 *
 *               compile with "Make brain"
 * ----------------------------------------------------------------------------
 */ 

#include "globals.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define K_DIV 100
#define DELAY 100
#define THRESH 10
#define ARRAY_LEN 100
#define MAX_EPOCHS 50

struct motor_command {
    int8_t left_speed;
    int8_t right_speed;
};

struct neuron_hidden{
    double input_l;
    double input_r;
    double weight_l;
    double weight_r;
    double bias;
    double output;
    //double o1_wt;
    //double o2_wt;
};

struct neuron_output{
    double input_1;
    double input_2;
    double input_3;
    double weight_1;
    double weight_2;
    double weight_3;
    double bias;
    double output;
};

void motor(struct motor_command);

int8_t do_PID(int16_t meas_cur, int16_t meas_prev, int16_t target, int16_t Kp, int16_t Kd);

struct motor_command compute_proportional(uint8_t left, uint8_t right);
struct motor_command compute_neural_network(uint8_t left, uint8_t right);
struct motor_command turn_off();

void compute_neuron_h(struct neuron_hidden * h);
void compute_neuron_o(struct neuron_output * o);

void randomize_h(struct neuron_hidden * h);
void randomize_o(struct neuron_output * o);
void gather_data(uint8_t left, uint8_t right);
void tilt_to_win();

void print_neuron_h(struct neuron_hidden * h);
void print_neuron_o(struct neuron_output * o);
void display_neurons();
void shuffle_set();

void do_training();
struct neuron_output get_output_training(struct neuron_output o, double target, double *bias_deriv);
struct neuron_hidden get_hidden_training(struct neuron_hidden h, double o1_bias_deriv, double o2_bias_deriv, double o1_wt, double o2_wt);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 global variables                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //PID variables (left)
    int16_t targetL = 195;
    int16_t KdL = 0;//-5;
    int16_t KpL = 100;
   
    //PID variables (right)
    int16_t targetR = 195;
    int16_t KdR = 0;//-5;
    int16_t KpR = 100;
   
    //last measurements
    int16_t last_meas_l = 0;
    int16_t last_meas_r = 0;

    //neurons
    struct neuron_hidden h1;
    struct neuron_hidden h2;
    struct neuron_hidden h3;
    struct neuron_output o1;    //output left
    struct neuron_output o2;    //output right
   
    //arrays to store data values
    uint8_t left_data[ARRAY_LEN] = {0};
    uint8_t right_data[ARRAY_LEN] = {0};
    uint8_t epochs = 0;
   
    //hyperparameters
    double alpha = 0.7;
   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 function main                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {
    init();  //initialize board hardware

    struct motor_command speed_control;

    int16_t meas_l = 0;
    int16_t meas_r = 0;
    uint8_t state = 0;
    uint8_t stall = 0;
    uint8_t tired = 0;
    
    randomize_h(&h1);
    randomize_h(&h2);
    randomize_h(&h3);
    randomize_o(&o1);
    randomize_o(&o2);
    
    while(1) {      
        //button debouncer
        if(get_btn()){
            if(stall == 0){
                stall = 1;
                //increment state
                state++;
                clear_screen();
            }
        }
        else{
            stall = 0;
        }
    
        //gather and print data
        meas_l = analog(2);
        meas_r = analog(3);
        
//        lcd_cursor(0,1);
//        print_num(meas_l);
//        lcd_cursor(5,1);
//        print_num(meas_r);

        //CPE student rolls the 'worst FSM known to man', asked to leave 416
        
        //PROPORTIONAL CONTROLLER STATE
        if(state == 0){
            lcd_cursor(0,0);
            print_string("PROPRTNL");
            speed_control = compute_proportional(meas_l, meas_r);
            motor(speed_control);
        }
        //DATA COLLECTION STATE
        else if(state == 1){
            lcd_cursor(0,0);
            print_string("DATA    ");
            speed_control = turn_off();
            motor(speed_control);
            gather_data(meas_l, meas_r);
            _delay_ms(100);
            lcd_cursor(0,1);
            print_num(meas_l);
            lcd_cursor(5,1);
            print_num(meas_r);
            
        }
        //EPOCH STATE
        else if(state == 2){
            lcd_cursor(0,0);
            print_string("EPCHS  ");
            tilt_to_win();
            _delay_ms(100);
        }
        //yes, if you press the button 255 times then it will go back to proportional.
        //TRAINING STATE
        else if((state > 2) && (state % 2)){
            lcd_cursor(0,0);
            print_string("TRAIN   ");
            speed_control = turn_off();
            motor(speed_control);
            if(!tired){
                tired = 1;
                do_training();
            }
        }
        //NEURAL NET ACTIVE
        else{
            lcd_cursor(0,0);
            print_string("SENTIENT");
            //display_neurons();
            tired = 0;
            speed_control = compute_neural_network(meas_l, meas_r);
            motor(speed_control);
        }
        
        //built-in delay
        _delay_us(DELAY); 
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 motor command                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// num: 0 or 1 for left or right motor respectively
// speed: -100 to 100 (negative is reverse)
void motor(struct motor_command speed_control) {
    //left motor - 157 is full forward,  97 is full backwards
    //right motor - 97 is full forward, 157 is full backwards
    uint8_t drive_left = ((60*(speed_control.left_speed + 100))/(200))-30;
    uint8_t drive_right = ((60*(speed_control.right_speed + 100))/(200))-30;

    set_servo(0, 127 + drive_left);
    set_servo(1, 127 - drive_right);
}

//MAYBE COMBINE THESE TWO FUNCTIONS????

//PID parameters
//  Target - what the measured value should be
//  Kp - product constant
//  Kd - derivative constant
int8_t do_PID(int16_t meas_cur, int16_t meas_prev, int16_t target, int16_t Kp, int16_t Kd){
   int16_t e = target - meas_cur;
   int16_t e_prev = target - meas_prev;
   int16_t p = (Kp * e)/K_DIV;
   int16_t d = (Kd * (e-e_prev))/K_DIV;
   int16_t u = p + d;
   //scale u from -255<->255 to -100<->100
   if(u < -100) {
      u = -100;
   }
   else if(u > 100) {
      u = 100;
   }
   else {
   }
   return u;
}

//compute the outputs to the motor using PID function from input values
struct motor_command compute_proportional(uint8_t left, uint8_t right){

   struct motor_command speed_control;

   speed_control.left_speed = do_PID(left, last_meas_l, targetL, KpL, KdL);
   speed_control.right_speed = do_PID(right, last_meas_r, targetR, KpR, KdR);

   last_meas_l = left;
   last_meas_r = right;

   return speed_control;
}

struct motor_command turn_off(){

    struct motor_command off_mode;
    off_mode.left_speed = 0;
    off_mode.right_speed = 0;
    return off_mode;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                 setup state                                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//randomizes the weights and bias of a hidden neuron
void randomize_h(struct neuron_hidden * h){
    //randomize the weights and biases
    h->weight_l = ((double)rand())/RAND_MAX-0.5;
    h->weight_r = ((double)rand())/RAND_MAX-0.5;
    h->bias = ((double)rand())/RAND_MAX-0.5;
    
    //zero the rest
    h->input_l = 0;
    h->input_r = 0;
    h->output = 0;
    //h->o1_wt = 0;
    //h->o2_wt = 0;
}

//randomizes the weights and bias of an output neuron
void randomize_o(struct neuron_output * o){
    //randomize the weights and biases
    o->weight_1 = ((double)rand())/RAND_MAX-0.5;
    o->weight_2 = ((double)rand())/RAND_MAX-0.5;
    o->weight_3 = ((double)rand())/RAND_MAX-0.5;
    o->bias = ((double)rand())/RAND_MAX;
    
    //zero the rest
    o->input_1 = 0;
    o->input_2 = 0;
    o->input_3 = 0;
    o->output = 0;
}

//gathers input data. While here, will continuously write data to the array, overwriting it in a circular fashion.
//will display information about how much data has been collected, as well as the current values being read. 
void gather_data(uint8_t left, uint8_t right){
    static uint8_t index = 0;
    static uint8_t full = 0;
    
    //loop if array is full
    if(index >= 100){
        //index = 0;
        full = 1;
        return;
    }
    lcd_cursor(5,0);
    //update values
    if(!full){
        print_num(index);
        left_data[index] = left;
        right_data[index] = right;
    }
    else{
        print_num(100);
    }

    index++;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              computation state                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//computes the output of the entire neural net from a set of input values
//updates the input and output values in each neural layer
//stores the output values in the motor_command struct, then returns it
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

//computes the output of a hidden neuron from its stored values
//stores the output value in the neuron's output variable
void compute_neuron_h(struct neuron_hidden * h){
    double net = (h->input_l * h->weight_l) + (h->input_r * h->weight_r) + h->bias;
    h->output = 1.0/(1.0 + exp(0-net));
}

//computes the output of an output neuron from its stored values
//stores the output value in the neuron's output variable
void compute_neuron_o(struct neuron_output * o){
    double net = (o->input_1 * o->weight_1) + (o->input_2 * o->weight_2) + (o->input_3 * o->weight_3) + o->bias;
    o->output = 1.0/(1.0 + exp(0-net));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               training state                                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void do_training() {
    //left array is left_data
    //right array is right_data
    for(uint8_t epoch = 0; epoch < epochs; epoch++) {
        lcd_cursor(6,0);
        print_num(epoch);
        for(uint8_t example = 0; example < ARRAY_LEN; example++) {
            struct neuron_hidden new_h[4]; //new_h[0] is unused
            struct neuron_output new_o[3]; //new_o[0] is unused
            
            //Step 1: run the neural network and p controller on the example
            compute_neural_network(left_data[example], right_data[example]);
            struct motor_command p_output = compute_proportional(left_data[example], right_data[example]);
            
            double o1_target = p_output.left_speed;
            double o2_target = p_output.right_speed;
            o1_target = (o1_target + 100)/200;
            o2_target = (o2_target + 100)/200;
            
            //Step 2: calculate the error
            //error function is ((target/output)^2)/2
            //double o1_error = ((o1_target - o1.output)*(o1_target - o1.output))/2;
            //double o2_error = ((o2_target - o2.output)*(o2_target - o2.output))/2;
            //double total_error = o1_error + o2_error;
            
            //Step 3: calculate new output neuron params
            double o1_bias_deriv;
            double o2_bias_deriv;
            new_o[1] = get_output_training(o1, o1_target, &o1_bias_deriv);
            new_o[2] = get_output_training(o2, o2_target, &o2_bias_deriv);
            
            //Step 4: calculate hidden neuron params
            new_h[1] = get_hidden_training(h1, o1_bias_deriv, o2_bias_deriv, new_o[1].weight_1, new_o[2].weight_1);
            new_h[2] = get_hidden_training(h2, o1_bias_deriv, o2_bias_deriv, new_o[1].weight_2, new_o[2].weight_2);
            new_h[3] = get_hidden_training(h3, o1_bias_deriv, o2_bias_deriv, new_o[1].weight_3, new_o[2].weight_3);
            
            //Final step - update all the parameters
            memcpy(&o1, &new_o[1], sizeof(struct neuron_output));
            memcpy(&o2, &new_o[2], sizeof(struct neuron_output));
            memcpy(&h1, &new_h[1], sizeof(struct neuron_hidden));
            memcpy(&h2, &new_h[2], sizeof(struct neuron_hidden));
            memcpy(&h3, &new_h[3], sizeof(struct neuron_hidden));
        }
    
        //shuffle around the training data:
        shuffle_set(); 
        //linearly decrease training rate
        //might have to comment this to work
        alpha -= (0.01*epoch);
    }
    
}

struct neuron_output get_output_training(struct neuron_output o, double target, double *bias_deriv) {
    struct neuron_output new_o;
    memcpy(&new_o, &o, sizeof(struct neuron_output));
    double deriv;
    
    //weight 1
    deriv = (o.output - target)*(o.output*(1 - o.output))*(o.input_1);
    new_o.weight_1 = o.weight_1 - (alpha * deriv);
    
    //weight 2
    deriv = (o.output - target)*(o.output*(1 - o.output))*(o.input_2);
    new_o.weight_2 = o.weight_2 - (alpha * deriv);
    
    //weight 3
    deriv = (o.output - target)*(o.output*(1 - o.output))*(o.input_3);
    new_o.weight_3 = o.weight_3 - (alpha * deriv);
    
    //bias
    deriv = (o.output - target)*(o.output*(1 - o.output));
    *bias_deriv = deriv;
    new_o.bias = o.bias - (alpha * deriv);
    
    return new_o;
}


struct neuron_hidden get_hidden_training(struct neuron_hidden h, double o1_bias_deriv, double o2_bias_deriv, double o1_wt, double o2_wt) {
    struct neuron_hidden new_h;
    memcpy(&new_h, &h, sizeof(struct neuron_hidden));
    double deriv;
    
    //weight l
    deriv = (o1_bias_deriv * (o1_wt) + o2_bias_deriv * (o2_wt)) * (h.output*(1 - h.output)) * (h.input_l);
    new_h.weight_l = h.weight_l - (alpha * deriv);
    
    //weight r
    deriv = (o1_bias_deriv * (o1_wt) + o2_bias_deriv * (o2_wt)) * (h.output*(1 - h.output)) * (h.input_r);
    new_h.weight_r = h.weight_r - (alpha * deriv);
    
    //bias
    deriv = (o1_bias_deriv * (o1_wt) + o2_bias_deriv * (o2_wt)) * (h.output*(1 - h.output));
    new_h.bias = h.bias - (alpha * deriv);
    
    return new_h;
}

//goofy ahh function, allows you to tilt the board to set the number of epochs (fun!)
//if we have time and storage space, make it so it in/decreases more when tilted more
void tilt_to_win(){

    uint8_t y_accel = 0;
    
    //get the current tilt of the board
    y_accel = get_accel_y();
    
    //tilt right
    if((128 < y_accel) && (y_accel < 255)){
        if(epochs < MAX_EPOCHS){
            epochs++;
        }
    }
    //tilt left
    else{
        if(epochs > 0){
            epochs--;
        }
    }
    
    lcd_cursor(6,0);
    print_num(epochs);
}

//swaps each element in the dataset arrays randomly with another element, including itself.
//should hopefully help improve data
void shuffle_set(){

    uint8_t indexnum = 0;
    uint8_t templ = 0;
    uint8_t tempr = 0;
    
    //randomly swap all elements in the arrays
    for(int i = 0; i < ARRAY_LEN; i++){
        indexnum = (uint8_t)((rand()/RAND_MAX)*100);
        templ = left_data[i];
        tempr = right_data[i];
        
        left_data[i] = left_data[indexnum];
        right_data[i] = right_data[indexnum];
        
        left_data[indexnum] = templ;
        right_data[indexnum] = tempr;    
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              display functions                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//prints out the values of all weights and biases of all neurons. Basically freezes the processor, so use with caution.
void display_neurons(){

    clear_screen();

    lcd_cursor(0,0);
    print_string("h1");
    print_neuron_h(&h1);

    lcd_cursor(0,0);
    print_string("h2");
    print_neuron_h(&h2);

    lcd_cursor(0,0);
    print_string("h3");
    print_neuron_h(&h3);

    lcd_cursor(0,0);
    print_string("o1");
    print_neuron_o(&o1);

    lcd_cursor(0,0);
    print_string("o2");
    print_neuron_o(&o2);
}

void print_neuron_h(struct neuron_hidden * h){

    lcd_cursor(3,0);
    print_string("WT-L");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * h->weight_l));
    _delay_ms(500);
    lcd_cursor(3,0);
    print_string("WT-R");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * h->weight_r));
    _delay_ms(500);
    lcd_cursor(3,0);
    print_string("BIAS");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * h->bias));
    _delay_ms(500);
}

void print_neuron_o(struct neuron_output * o){

    lcd_cursor(3,0);
    print_string("WT-1");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * o->weight_1));
    _delay_ms(1200);
    lcd_cursor(3,0);
    print_string("WT-2");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * o->weight_2));
    _delay_ms(1200);
    lcd_cursor(3,0);
    print_string("WT-3");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * o->weight_3));
    _delay_ms(1200);
    lcd_cursor(3,0);
    print_string("BIAS");
    lcd_cursor(0,1);
    print_num((uint16_t)(1000 * o->bias));
    _delay_ms(1200);
}
