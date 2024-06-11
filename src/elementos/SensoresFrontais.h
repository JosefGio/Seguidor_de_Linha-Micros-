#ifndef SENSORES_FRONTAIS_H
#define SENSORES_FRONTAIS_H

#include <stdint.h>

void atualizar_leitura(void);

struct SensoresFrontais {
    int16_t (*posicao_media)(void);
} sensores_frontais;


typedef int sensor_frontal_t;

typedef struct {
  uint8_t (*valor_lido)(sensor_frontal_t sensor);
} IDriverSensoresFrontais;

void SensoresFrontais_init_driver(IDriverSensoresFrontais driver);

#endif