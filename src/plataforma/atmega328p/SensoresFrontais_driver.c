#include <stdint.h>
#include <avr/interrupt.h>
#include "plataforma/atmega328p/libs/ADC.h"
#include "elementos/SensoresFrontais.h"
#include "plataforma/drivers_setup.h"

uint8_t sensores_de_tensao[7];

static uint8_t _valor_lido(sensor_frontal_t sensor) {
    return (sensores_de_tensao[sensor]);
}


static void tratar_leitura_do_ADC(void)
{/* inicializo no setup na fun��o calibration e em seguida toda
     * vez que ocorre uma conver��o a interrup��o do AD ocorre
     * e ent�o esta fun��o � chamada pelo vetor de interrup��o
     * do AD, obtendo os dados da convers�o em "paralelo" � rotina */
    
    /* Leio primeiro o default que seria o primeiro canal
     * e em seguida fa�o uma l�gica circular de leitura dos canais */
    
  static uint8_t estado = 10;
  
  switch (estado) {
      
    case 0:
      estado = 1;
      sensores_de_tensao[0] = adc_read_service();
      adc_conversion_ch_service(3);
      break;
        
    case 1:
      estado = 2;
      sensores_de_tensao[1] = adc_read_service();
      adc_conversion_ch_service(4);
      break;

    case 2:
      estado = 3;
      sensores_de_tensao[2] = adc_read_service();
      adc_conversion_ch_service(5);
      break;

    case 3:
      estado = 4;
      sensores_de_tensao[3] = adc_read_service();
      adc_conversion_ch_service(6);
      break;

    case 4:
      estado = 5;
      sensores_de_tensao[4] = adc_read_service();
      adc_conversion_ch_service(7);
      break;

    case 5:
      estado = 6;
      sensores_de_tensao[5] = adc_read_service();
      adc_conversion_ch_service(1);
      break;

    case 6:
      estado = 0;
      sensores_de_tensao[6] = adc_read_service();
      adc_conversion_ch_service(2);
      break;

    default:
      estado = 0;
      adc_conversion_ch_service(2);
      sensores_de_tensao[0] = adc_read_service();
      break; 
  }    
}


void setup_driver_SensoresFrontais(void) {
    IDriverSensoresFrontais driver = {
        .valor_lido = _valor_lido
    };

    SensoresFrontais_init_driver(driver);

    tratar_leitura_do_ADC();
}


ISR(ADC_vect) {
    tratar_leitura_do_ADC();
}