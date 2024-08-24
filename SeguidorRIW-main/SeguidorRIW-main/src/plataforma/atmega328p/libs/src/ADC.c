#include <avr/io.h>
#include "../ADC.h"
// #include "plataforma/atmega328p/libs/ADC.h"

void adc_conversion_ch_service(unsigned char channel)
{
  ADMUX &= 0xf0;
  ADMUX |= (channel & 0x0f);
  
  ADCSRA |= 0x40;
}

uint16_t adc_read_service(void)
{
  uint16_t dado = (ADCH<<8) | (ADCL);
  return dado;
}
