#include <avr/io.h>
#include "plataforma/atmega328p/libs/ADC.h"

void adc_conversion_ch_service(unsigned char channel)
{
  ADMUX &= 0xf0;
  ADMUX |= (channel & 0x0f);
  
  ADCSRA |= 0x40;
}

uint8_t adc_read_service(void)
{
  unsigned char dado = ADCH;
  return dado;
}
