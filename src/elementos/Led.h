#ifndef LED_H
#define LED_H

struct Led {
  void (*ligar)(void);
  void (*desligar)(void);
  void (*alternar) (void);
} led_offboard;

typedef struct {
  void (*ligar)(void);
  void (*desligar)(void);
  void (*alternar) (void);
} IDriverLed;

void Led_Offboard_init_driver(IDriverLed driver);

#endif