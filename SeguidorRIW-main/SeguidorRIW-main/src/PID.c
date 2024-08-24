#include "PID.h"
#include "elementos/UART.h" //apagar apos teste
#include <stdio.h>



#define ANGULAR 0
#define LINEAR  1


void acoes_de_controle(float *proporcional, float *integral, float *derivativo, int16_t error, int8_t dado);
int16_t filtro_derivativo(int16_t derivacao);

char string[20];



float kp_ang = {22},    // aumentar se o robô não estiver fazendo curva // 13.0 // 30 valor bom
    ki_ang = {0.0000004},// aumentar se o robô estiver oscilando muito // 0.00112 0.00645 //0.00250 valor bom 
    kd_ang = {0.830};  //  aumentar se na passagem de reta para curva e vice-versay o robô balançar muito // 0.185 0.295 // 0.320 valor

float kp_motor = {60.0},
    kd_motor = {0.00040},
    ki_motor = {0.000};

/*Sera necessário alterar as constantes do controle PID*/

float PID_angular(float error) /* Algoritmo de controle PID usando os sensores frontais */
{   
    static float P, I, D, previous_error, response;

    P = kp_ang * error;
    I = ki_ang * (I+error); 
    D = kd_ang * (error - previous_error);
    previous_error = error;

    response = P+I+D;
    return response; 

} /* end PID */

float PID_motor(float error) /* Algoritmo de controle PID usando os sensores frontais */
{   
    static float P, I, D, previous_error, response;

    P = kp_motor * error;
    I = ki_motor * (I+error); 
    D = kd_motor * (error - previous_error);
    previous_error = error;

    response = P+I+D;
    return response; 

} /* end PID */