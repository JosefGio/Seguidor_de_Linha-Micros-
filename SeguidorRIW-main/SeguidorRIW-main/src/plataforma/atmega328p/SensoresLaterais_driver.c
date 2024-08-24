#include <stdbool.h>
#include <avr/io.h>
#include "plataforma/drivers_setup.h"
#include "elementos/SensoresLaterais.h"

#define REG_SENSORES_LATERAIS  PINB
#define SENSOR_DE_CURVA        PB3
#define SENSOR_DE_PARADA       PB4

bool ler_sensor(sensor_lateral_t sensor) {
  if (sensor == S_CURVA)
    return !(REG_SENSORES_LATERAIS & (1<<SENSOR_DE_CURVA));    
  else if (sensor == S_PARADA)
    return !(REG_SENSORES_LATERAIS & (1<<SENSOR_DE_PARADA));

  return 0;
}

void setup_driver_SensoresLaterais(void) {
  IDriverSensoresLaterais driver = {
    .ler_sensor = ler_sensor
  };

  SensoresLaterais_init_driver(driver);
}