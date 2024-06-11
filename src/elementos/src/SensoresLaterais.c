#include <stdbool.h>
#include "elementos/SensoresLaterais.h"

IDriverSensoresLaterais _driver;

void SensoresLaterais_init_driver(IDriverSensoresLaterais driver) {
  _driver = driver;
}

sensor_lateral_t _obter_leitura(void) {
  bool sensor_curva_lido = _driver.ler_sensor(S_CURVA);
  bool sensor_parada_lido = _driver.ler_sensor(S_PARADA);

  if (sensor_curva_lido & sensor_parada_lido)
    return S_CURVA_E_PARADA;
  else if (sensor_curva_lido)
    return S_CURVA;
  else if (sensor_parada_lido)
    return S_PARADA;
  else
    return NENHUM;
}

struct SensoresLaterais sensores_laterais = {
  .obter_leitura = _obter_leitura
};
