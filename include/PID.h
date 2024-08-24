#ifndef PID_H
#define PID_H

#define FRONTAIS 0
#define ENCODERS 1
#include <stdint.h>

float PID_angular(float error); /* Algoritmo de controle PID usando os sensores frontais */
float PID_motor(float error);

#endif