#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void) {
   init();  //initialize board hardware
   led_on(1);
   return 0;
}
