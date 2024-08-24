#ifndef SENSORES_LATERAIS_H
#define SENSORES_LATERAIS_H

#include <stdbool.h>

typedef enum {
  S_CURVA=1, S_PARADA, S_CURVA_E_PARADA, NENHUM
} sensor_lateral_t;

struct SensoresLaterais {
  sensor_lateral_t (*obter_leitura)(void);
} sensores_laterais;


// HAL
typedef struct {
  bool (*ler_sensor)(sensor_lateral_t sensor);
} IDriverSensoresLaterais;

void SensoresLaterais_init_driver(IDriverSensoresLaterais driver);

#endif