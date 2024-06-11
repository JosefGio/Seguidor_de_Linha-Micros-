/*#include <stdlib.h>
#include "Running.h"
#include "elementos/Encoders.h"

#define NUMERO_DE_TRECHOS 23
#define RETA 500

extern int16_t setpoint_x_PWM[NUMERO_DE_TRECHOS];
extern uint8_t _trecho;

uint16_t pulsos_pista[NUMERO_DE_TRECHOS];
uint16_t pulso_atual = 0, pulso_anterior = 0, pulsos_faltando = 0;
uint16_t pulsos_necessarios[NUMERO_DE_TRECHOS]; //quantidade de pulsos a serem percorridos para que o robô comece a frear
static   velocidades_t velocidade;

int16_t avg_pulsos();
void run(void);

void run(void){  
    static uint16_t v = 0, v0 = 0;
    const float a = 4.6 * 981 ;     //0.46 * 9.81 *1000

    v =  (3.018 * setpoint_x_PWM[_trecho]) - 244.8;   //velocidade ideal no trecho
    v0 = velocidade.linear;                         //velocidade atual    
    pulsos_necessarios[_trecho] = ((((v0 * v0)  - (v * v)) / (2*a))) / (1.765);    //pulsos
    pulso_atual =  avg_pulsos() - pulso_anterior;   //quanto ja percorreu
    pulsos_faltando = pulsos_pista[_trecho] - pulso_atual;

    if (pulsos_faltando > pulsos_necessarios[_trecho] && setpoint_x_PWM[_trecho] == RETA){    //preciso saber que eh uma reta para executar este if
        _trecho++;  //mudo para o proximo trecho
        pulso_anterior = pulso_atual;
        pulso_atual = 0;    //zero o pulso por mudar de trecho
    }
}

int16_t avg_pulsos()
{
    return(abs((encoder_direito.obter_pulsos() - encoder_esquerdo.obter_pulsos()) / 2)); 
}*/