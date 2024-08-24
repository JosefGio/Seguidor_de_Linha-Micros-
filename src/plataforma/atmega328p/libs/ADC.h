#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_conversion_ch_service(unsigned char channel);
uint16_t adc_read_service(void);

#endif