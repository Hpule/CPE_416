#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void) {
   init();  //initialize board hardware
   led_on(LED0_PIN); 
   led_on(LED1_PIN);
   return 0;
}
