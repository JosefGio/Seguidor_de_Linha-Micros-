#ifndef BITS_FIELD
#define BITS_FIELD

#include <stdio.h>
#include <stdint.h>

typedef struct
{
  uint16_t    PWM_front_motors     : 10;  
  int16_t    PWM_right            : 12;
  int16_t    PWM_left             : 12;
} bits_field;

bits_field variable;

typedef struct
{
  uint16_t counter : 10; //contador
  uint16_t compare : 10; //comparador de tempo
} Timer;

Timer max_timer1;
Timer max_timer3;
Timer max_timer2;
Timer max_timer4;

#endif