#include "PID.h"
#include "elementos/UART.h" //apagar apos teste
#include <stdio.h>

#define ANGULAR 0
#define LINEAR  1


void acoes_de_controle(float *proporcional, float *integral, float *derivativo, int16_t error, int8_t dado);
int16_t filtro_derivativo(int16_t derivacao);

char string[20];


//#define OPEN_LOOP
#define CLOSED_LOOP

#ifdef CLOSED_LOOP
    float kp_ang = {13.0},    // aumentar se o robô não estiver fazendo curva
        ki_ang = {0.00000},// aumentar se o robô estiver oscilando muito
        kd_ang = {0.090};  // aumentar se na passagem de reta para curva e vice-versay o robô balançar muito

    float kp_line = {60.0},
        kd_line = {0.00040},
        ki_line = {0.000};
#endif

#ifdef OPEN_LOOP
    float kp_ang = {13.5},    // aumentar se o robô não estiver fazendo curva
        ki_ang = {0.000000},// aumentar se o robô estiver oscilando muito
        kd_ang = {0.14000};  // aumentar se na passagem de reta para curva e vice-versay o robô balançar muito

    float kp_line = {0},
        kd_line = {0.00000},
        ki_line = {0.000};
#endif

/*Sera necessário alterar as constantes do controle PID*/

int16_t PID(int16_t error, int8_t dado) /* Algoritmo de controle PID usando os sensores frontais */
{   
    int16_t correcao = 0;

    static float  proporcional = 0;
    static float  integral     = 0;
    static float  derivativo   = 0;

    acoes_de_controle(&proporcional, &integral, &derivativo, error, dado);
    correcao = (proporcional + integral + derivativo);
    
    return correcao; 

} /* end PID */

void acoes_de_controle(float *proporcional, float *integral, float *derivativo, int16_t error, int8_t dado)
{
    static int16_t erroAnterior_angular = 0, erroAnterior_linear = 0;
    static int32_t acao_integrativa_angular = 0, acao_integrativa_linear = 0; 
    static int16_t acao_derivativa_angular = 0, acao_derivativa_linear   = 0, acao_derivativa_angular_filtrada = 0, acao_derivativa_linear_filtrada = 0;

    if(dado == ANGULAR)
    {   
        if(!error)
            acao_integrativa_angular = 0;
        
        acao_integrativa_angular += error;
        acao_derivativa_angular = error - erroAnterior_angular;
        erroAnterior_angular    = error;  

        acao_derivativa_angular_filtrada = filtro_derivativo(acao_derivativa_angular);
    
        //sprintf(string, "%d\t%d\n", acao_derivativa_angular, acao_derivativa_angular_filtrada);
        //uart.enviar_string((int8_t *)string);   //apagar apos teste

        
        *proporcional = (kp_ang*error);  
        *integral     = (ki_ang*acao_integrativa_angular);
        *derivativo   = (kd_ang*acao_derivativa_angular_filtrada);
    }

    else if(dado == LINEAR)
    {   
        if(!error)
            acao_integrativa_linear = 0;
        acao_integrativa_linear += error;
        acao_derivativa_linear = error - erroAnterior_linear;
        erroAnterior_linear    = error; 
        acao_derivativa_linear_filtrada = filtro_derivativo(acao_derivativa_linear);

        *proporcional = (kp_line*error);  
        *integral     = (ki_line*acao_integrativa_linear);
        *derivativo   = (kd_line*acao_derivativa_linear_filtrada);
    }
    
}


int16_t filtro_derivativo(int16_t derivacao)
{
    #define NUMERO_DE_AMOSTRAS 20
    static int16_t erro_derivativo[NUMERO_DE_AMOSTRAS] = {0};
    int16_t acumulador = 0;

    for (int i = NUMERO_DE_AMOSTRAS - 1; i > 0; i--)
        erro_derivativo[i] = erro_derivativo[i - 1];

    erro_derivativo[0] = derivacao;

    for (int i = 0; i < NUMERO_DE_AMOSTRAS; i++)
    {
        acumulador += erro_derivativo[i];
    }

    return acumulador / NUMERO_DE_AMOSTRAS;
}