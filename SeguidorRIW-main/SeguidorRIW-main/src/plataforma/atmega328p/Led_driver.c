#include <stdbool.h>
#include <avr/io.h>
#include "elementos/Led.h"
#include "plataforma/drivers_setup.h"

#define REG_LED    PORTC
#define LED        PC0

static void high (void)
{
  REG_LED |= (1<<LED);
}

static void low (void)
{
  REG_LED &= ~(1<<LED);
}

static void toggle (void)
{
  REG_LED ^= (1<<LED);
}

void setup_driver_Led_offboard(void)
{
  IDriverLed driver = {
    .ligar = high,
    .desligar = low,
    .alternar = toggle
  };

  Led_Offboard_init_driver(driver);
}