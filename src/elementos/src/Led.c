#include <avr/io.h>
#include "elementos/Led.h"

struct Led led_offboard;

void Led_Offboard_init_driver(IDriverLed driver) {
  led_offboard.ligar = driver.ligar;
  led_offboard.desligar = driver.desligar;
  led_offboard.alternar = driver.alternar;
}