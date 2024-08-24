#ifndef SENSORS_H
#define SENSORS_H
#include <stdint.h>

uint8_t sensores_linha [5];
int16_t obter_posicao_frontal_relativa(void);
int16_t calculo_delta_speed(bool selecao);
void calculo_de_velocidade(void);
int16_t sensors_encoder(bool selecao);
void volta_pra_pista(int16_t posicao_relativa);

#endif 