
/*Name: Christine Choi and Angelika Canete
//Lab 3 part 2
//Description:

The program implements a line following robot using back propagation.
The robot using a PID equation to compute the target motor outputs, and
then trains weights and biases to match the target.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_EYE 0
#define RIGHT_EYE 1

#define IS_BETWEEN(x, a, b) ((x) >= (a) && (x) <= (b))
#define K_P 0.2
#define K_D 0.001
#define K_I 0.05
#define DEFAULT_SPEED 20
#define NUM_OF_ERROR_SAMPLES 5
#define NUM_OF_COLLECTED_SAMPLES 100
#define MOTOR_STABLE 127        //motors do not move at 127
#define LEARN_RATE .137


struct motor_command
{
        double left;
        double right;
} motor_command;

struct pure_data
{
    uint8_t left;
    uint8_t right;
} pure_data;

struct pure_data sensor_val[NUM_OF_COLLECTED_SAMPLES];
int sample_count = 0;

struct neural_node {
    double w1;
    double w2;
    double w3;
    double bias;
} neural_node;

struct neural_node network[5] = {0};

    double h1_out = 0;
    double h2_out = 0;
    double h3_out = 0;
    double o1_out = 0;
    double o2_out = 0;

void motor(uint8_t num, int8_t speed)
{ //num will be 0 or 1 corresponding to the left or right motor
  // speed will be a number from -100 (full speed reverse) to
  //+100 (full speed forward)

        uint8_t adjusted_speed;
        if (num == 0)
        {
                adjusted_speed = (speed*3)/10 + MOTOR_STABLE;
                set_servo(RIGHT_MOTOR, adjusted_speed);
        }
        else
        {
                adjusted_speed = (-speed*3)/10 + MOTOR_STABLE;
                set_servo(LEFT_MOTOR, adjusted_speed);
        }
}
//prevents button debouncing
int button_debounce()
{
    int button = 0;
    while(get_btn())
    {
        button = 1;
    }
    return button;
}

//acts as a delay_ms function that also checks for a button press
u16 button_delay_check(u16 loop)
{
    //acts as a delay_ms function that also checks for a button press

        u16 count = 0;
        int button_flag = 0;
        while (count < loop)
        {
            if (get_btn()==1)
            {
                button_flag = 1;
            }
            _delay_ms(1);
            count ++;
        }
        return button_flag;
}

void add_to_error_array(int added_num, int error_samples[NUM_OF_ERROR_SAMPLES])
{
    /*Add to an array for derivative in PID*/
    for (int i = NUM_OF_ERROR_SAMPLES - 1; i > 0; i--) {
        error_samples[i] = error_samples[i - 1];
    }

    error_samples[0] = added_num;
}

float calculate_average_error(int error_samples[NUM_OF_ERROR_SAMPLES])
{
    /*Calculate average error for derivative in PID*/
    int sum = 0;
    for (int i = 0; i < NUM_OF_ERROR_SAMPLES; i++) {
        sum += error_samples[i];
    }
   return (float)sum / NUM_OF_ERROR_SAMPLES;
}

//Changes a value to be between 0-1
double normalize(uint8_t value)
{
    double result = (value / 100.0);
    if (result < 0.0) {
        return 0.0;
    } else if (result > 1.0) {
        return 1.0;
    } else {
        return result;
    }
}
//changes a value to be between 0-100
double denormalize(double value)
{
    double result = (value * 100.0);
    if (result < 0.0) {
        return 0.0;
    } else if (result > 100.0) {
        return 100.0;
    } else {
        return result;
    }
}
//takes in left and right sensor values and outputs computed motor values by pid model
struct motor_command compute_proportional(uint8_t curr_left, uint8_t curr_right)
{
        struct motor_command curr_motor_command =  {0};
        static int error_samples[NUM_OF_ERROR_SAMPLES] = {0};

        static int prev_error = 0;
        float derivative = 0;

        //find derivative
        int error = curr_left - curr_right;
        add_to_error_array(error, error_samples);
        derivative = calculate_average_error(error_samples);

        //PID equation
        int leftMotorSpeed = 45 + K_P * error + K_I * (error + prev_error) + K_D * derivative;
        int rightMotorSpeed = 45 - K_P * error - K_I * (error + prev_error) - K_D * derivative;

        if (leftMotorSpeed < 0)
        {
            leftMotorSpeed = 0;
        }
        if (rightMotorSpeed < 0)
        {
            rightMotorSpeed = 0;
        }

        //set the motors
        curr_motor_command.left = normalize(leftMotorSpeed);
        curr_motor_command.right = normalize(rightMotorSpeed);

        prev_error = error;

        return curr_motor_command;

}
//randomize weights on initialization
void network_init()
{
    network[0].w1 =(double) rand()/ RAND_MAX;
    network[0].w2 = (double)rand()/ RAND_MAX;
    network[0].bias = (double)rand()/ RAND_MAX;

    network[1].w1 =(double) rand()/ RAND_MAX;
    network[1].w2 = (double)rand()/ RAND_MAX;
    network[1].bias = (double)rand()/ RAND_MAX;

    network[2].w1 = (double)rand()/ RAND_MAX;
    network[2].w2 = (double)rand()/ RAND_MAX;
    network[2].bias = (double)rand()/ RAND_MAX;


    //output layer weights
    network[3].w1 = (double)rand()/ RAND_MAX;
    network[3].w2 = (double)rand()/ RAND_MAX;
    network[3].w3 = (double)rand()/ RAND_MAX;
    network[3].bias = (double)rand()/ RAND_MAX;

    network[4].w1 = (double)rand()/ RAND_MAX;
    network[4].w2 = (double)rand()/ RAND_MAX;
    network[4].w3 = (double)rand()/ RAND_MAX;
    network[4].bias = (double)rand()/ RAND_MAX;


}

int data_collection()
{/*
Collect sensor readings and input array into a global struct
*/
        int curr_left = 0;
        int curr_right = 0;
        // sensor values

        for (sample_count = 0; sample_count < NUM_OF_COLLECTED_SAMPLES; sample_count++)
        {
                curr_left = (analog(LEFT_EYE));
                curr_right = (analog(RIGHT_EYE));

                struct pure_data curr_reading;
                curr_reading.left = curr_left;
                curr_reading.right = curr_right;

                sensor_val[sample_count] = curr_reading;

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
        }
        clear_screen();
        lcd_cursor(0, 0);
        print_string("victory!");
    return sample_count;

}

void line_seeking_PID()
{/*
Measure the outside of the lines
If both sensors see the same value, assume they are on the correct side of the line
If a sensor does not see the corrrect value,  correct by the PDI
*/
    int curr_left = 0;
    int curr_right = 0;
    int prev_error = 0;
    int analog_samples[NUM_OF_ERROR_SAMPLES] = {0};
    int error = 0;
    float derivative = 0;
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

        while (true)
        {
                if (button_debounce())
                {
                    break;
                }
                // sensor values
                curr_left = (analog(LEFT_EYE));
                curr_right = (analog(RIGHT_EYE));

                //find derivative
                error = curr_left - curr_right;
                add_to_error_array(error, analog_samples);
                derivative = calculate_average_error(analog_samples);

                //PID equation
                leftMotorSpeed = 45 + K_P * error + K_I * (error + prev_error) + K_D * derivative;
                rightMotorSpeed = 45 - K_P * error - K_I * (error + prev_error) - K_D * derivative;

                if (leftMotorSpeed < 0)
                {
                    leftMotorSpeed = 0;
                }
                if (rightMotorSpeed < 0)
                {
                    rightMotorSpeed = 0;
                }

                //set the motors
                motor(LEFT_MOTOR, leftMotorSpeed);
                motor(RIGHT_MOTOR, rightMotorSpeed);

                prev_error = error;
    }
}


void delay(u16 loop)
{
        u16 count = 0;
        while (count < loop)
        {
                _delay_ms(1);
                count ++;
        }
}


int get_training_itertions()
{
    //receive user input by tilting
    clear_screen();
    lcd_cursor(0, 0);
    print_string("<=-  +=>");
    int iterations = 0;

    while(1)
    {
        if ((get_accel_y() > 30 && get_accel_y()< 70 ))
        {//to the left
            iterations--;
        }
        if ( (get_accel_y() > 200 && get_accel_y()  < 250 ))
        {//to the right
            iterations++;
        }
        if ((get_accel_x() > 200 && get_accel_x()< 230 ))
        {// move down
            iterations = iterations - 10;
        }
        if ( (get_accel_x() > 20 && get_accel_x() < 70 ) )
        {//move up
            iterations = iterations + 10;
        }

        if (iterations < 0)
        {//no negatives!
                iterations = 0;
        }

        lcd_cursor(0, 1);
        print_num(iterations);
        print_string("       ");
        if (button_delay_check(50))
        {
                return iterations;
        }

    }
    return 0;

}


void train_neural_network(struct motor_command target, struct motor_command out, int sensor_left, int sensor_right)
{
    //receives datapoints captured outputs update neural network

    double db_right = normalize(sensor_right);
    double db_left = normalize(sensor_left);

    //----hidden----------------------
    double new0_w1 = network[0].w1 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * db_left));

    double new0_w2 = network[0].w2 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * db_right));

    double new0_bias = network[0].bias - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w1)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w1))
    * (h1_out * (1-h1_out)) * -1));

    double new1_w1 = network[1].w1 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * db_left));

    double new1_w2 = network[1].w2 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * db_right));

    double new1_bias = network[1].bias - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w2)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w2))
    * (h2_out * (1-h2_out)) * -1));

    double new2_w1 = network[2].w1 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * db_left));

    double new2_w2 = network[2].w2 - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * db_right));
    
    double new2_bias = network[2].bias - (LEARN_RATE *
    ((((out.left - target.left)* (out.left * (1 - out.left))* network[3].w3)
    + ((out.right - target.right)* (out.right * (1 - out.right))* network[4].w3))
    * (h3_out * (1-h3_out)) * -1));

    //----output----------------------
    double new3_w1 =  network[3].w1 - (LEARN_RATE *
    ((out.left - target.left)* (out.left * (1 - out.left))* (h1_out)));
    double new3_w2  =  network[3].w2 - (LEARN_RATE *
    ((out.left - target.left)* (out.left * (1 - out.left))* (h2_out)));
    double new3_w3 =  network[3].w3 - (LEARN_RATE *
    ((out.left - target.left)* (out.left * (1 - out.left))* (h3_out)));
    double new3_bias = network[3].bias - (LEARN_RATE *
    ((out.left - target.left)* (out.left * (1 - out.left))* (-1)));

    double new4_w1 =  network[4].w1 - (LEARN_RATE *
    ((out.right - target.right)* (out.right * (1 - out.right))* (h1_out)));
    double new4_w2 = network[4].w2 - (LEARN_RATE *
    ((out.right - target.right)* (out.right * (1 - out.right))* (h2_out)));
    double new4_w3 =  network[4].w3 - (LEARN_RATE *
    ((out.right - target.right)* (out.right * (1 - out.right))* (h3_out)));
    double new4_bias = network[4].bias - (LEARN_RATE *
    ((out.right - target.right)* (out.right * (1 - out.right))* (-1)));


    //update
    network[0].w1 = new0_w1;
    network[0].w2 = new0_w2;
    network[0].bias = new0_bias;

    network[1].w1 = new1_w1;
    network[1].w2 = new1_w2;
    network[1].bias = new1_bias;

    network[2].w1 = new2_w1;
    network[2].w2 = new2_w2;
    network[2].bias = new2_bias;

    network[3].w1 = new3_w1;
    network[3].w2 = new3_w2;
    network[3].w2 = new3_w3;
    network[3].bias = new3_bias;

    network[4].w1 = new4_w1;
    network[4].w2 = new4_w2;
    network[4].w2 = new4_w3;
    network[4].bias = new4_bias;

}
    //sigmoid calculation


struct motor_command compute_neural_network(uint8_t sensor_left, uint8_t sensor_right)
{
    double curr_left = normalize(sensor_left);
    double curr_right = normalize(sensor_right);

    //calculate net value
    double h1_net = ((curr_left * network[0].w1) + (curr_right * network[0].w2)) - network[0].bias;
    double h2_net = ((curr_left * network[1].w1) + (curr_right * network[1].w2)) - network[1].bias;
    double h3_net = ((curr_left * network[2].w1) + (curr_right * network[2].w2)) - network[2].bias;

    //sigmoid calculation
     h1_out = 1/(1 + exp(-(h1_net)));
     h2_out = 1/(1 + exp(-(h2_net)));
     h3_out = 1/(1 + exp(-(h3_net)));

    //outer layer equation
    double o1_net = ((h1_out * network[3].w1) + (h2_out * network[3].w2) + (h3_out * network[3].w3)) - network[3].bias;
    double o2_net = ((h1_out * network[4].w1) + (h2_out * network[4].w2) + (h3_out * network[4].w3)) - network[4].bias;

    //sigmoid calculation
     o1_out = (1/(1 + exp(-(o1_net))));
     o2_out = (1/(1 + exp(-(o2_net))));

    //outputs predicted motor values
    struct motor_command computed_nodes;
    computed_nodes.left = o1_out;
    computed_nodes.right = o2_out;


    return computed_nodes;
}

void run_motors(struct motor_command out)
{
    /*run motors based on struct*/
    int left = denormalize(out.left);
    int right = denormalize(out.right);

    lcd_cursor(0, 1);
    print_string("L");
    print_num(left);
    print_string("   ");
    lcd_cursor(4, 1);
    print_string("R");
    print_num(right);
    print_string("  ");

    motor(LEFT_MOTOR, left);
    motor(RIGHT_MOTOR, right);
}


enum state
{
    /*state machine */
INIT_STATE, PID_STATE, DATA_STATE, INPUT_STATE, TRAIN_STATE, FORWARD_STATE
};




int main(void)
{
    enum state current_state = INIT_STATE;
    int epochs = 0;
    int data_collected = 0;

   while(1)
   {
    switch(current_state)
    {
        case INIT_STATE:
        {
            init();  //initialize board hardware
            network_init();
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            current_state = PID_STATE;
        }
        break;

        case PID_STATE:
        {
            lcd_cursor(0,0);
            print_string("proporti");
            lcd_cursor(0,1);
            print_string("onal");

            line_seeking_PID();
            current_state = DATA_STATE;
        }
        break;

        case DATA_STATE:
        {   //collecting data
            _delay_ms(500);
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            data_collected = data_collection();

            current_state = INPUT_STATE;
        }
        break;

        case INPUT_STATE:
        {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            epochs = get_training_itertions();
            current_state = TRAIN_STATE;
        }
        break;

        case TRAIN_STATE:
        {
            clear_screen();
            lcd_cursor(0,0);
            print_string("Training: ");
            for (int i = 0; i < epochs; i++)
            {
                for (int j = 0; j < NUM_OF_COLLECTED_SAMPLES; j++)
                {
                    struct motor_command target = compute_proportional(sensor_val[j].left, sensor_val[j].right);
                    struct motor_command out = compute_neural_network(sensor_val[j].left, sensor_val[j].right);
                    train_neural_network(target, out, sensor_val[j].left, sensor_val[j].right);

                }
                lcd_cursor(0,1);
                print_num(i);
                print_string("   ");
            }
            current_state = FORWARD_STATE;
            clear_screen();
        }

        break;

        case FORWARD_STATE:
        {

            int left = analog(LEFT_EYE);
            int right = analog(RIGHT_EYE);

            struct motor_command current = compute_neural_network(left, right);
            run_motors(current);


            if (get_btn()==1)
            {
                delay(50);
                current_state = INPUT_STATE;
            }
            else
            {
                current_state = FORWARD_STATE;

            }
        }
        break;
    }
   }

return 0;
}