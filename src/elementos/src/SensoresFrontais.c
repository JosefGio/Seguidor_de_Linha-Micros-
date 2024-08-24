#include "elementos/SensoresFrontais.h"
#include "elementos/UART.h"
#include "elementos/Motores.h"
#include <stdbool.h>
#include <stdio.h>

#define NUM_SENSORES 7

// VALORES OBTIDOS NA CALIBRAÇÃO DOS SENSORES
#define VALOR_MAX 1023 //255 na calibracao
#define VALOR_MIN 0  //0 na calibracao

extern int8_t buffer[20];
static uint16_t sensores[NUM_SENSORES];
static uint16_t sensores_normalizados[NUM_SENSORES];
static IDriverSensoresFrontais _driver;
bool out_of_line = 0;

void atualizar_leitura(void);

void SensoresFrontais_init_driver(IDriverSensoresFrontais driver) {
    _driver = driver;
}

static float _posicao_media(void) {
    int16_t numerador = 0;
    int16_t denominador = 0; 
    float posicao_media = 0;
    int8_t posicoes[] = {30, 20, 10, 0, -10, -20, -30};

    for (int i = 0; i < NUM_SENSORES; i++) {
        sensores_normalizados[i] = VALOR_MAX - sensores[i];
        numerador += sensores_normalizados[i] * posicoes[i];
        denominador += sensores_normalizados[i];  
        //  sprintf(buffer, "%d\t", sensores[i]); //ver o peso dos sensores
        //  uart.enviar_string(buffer);      
    }
    
     uart.enviar_caractere('\n');

    //descomentar estes prints e ver o maximo menor valor e o minimo maior valor
    

    if(!denominador)
    {
        posicao_media = 0;
        out_of_line = 1;
    }
    else
    {
        posicao_media = (float)(numerador) / (float)(denominador);
        out_of_line = 0;
    }
    //  sprintf((char *)buffer, "%.2f\n", posicao_media);
    //  uart.enviar_string(buffer);
    return posicao_media;
}


void atualizar_leitura(void) {
    //======Estabelece o limiar da leitura dos sensores====//
    //função de correção da calibração
    uint16_t valor_lido;

    for (int i = 0; i < NUM_SENSORES; i++)
    {
        valor_lido = _driver.valor_lido(i);
                
        if (valor_lido < VALOR_MIN)
            sensores[i] = VALOR_MIN;        
        else if (valor_lido > VALOR_MAX)
            sensores[i] = VALOR_MAX;
        else            
            sensores[i] = valor_lido;
    }
}



struct SensoresFrontais sensores_frontais = {
    .posicao_media = _posicao_media
};