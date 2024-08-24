#include "elementos/UART.h"

static IDriverUART _driver;

void UART_init_driver(IDriverUART driver) {
    _driver = driver;
}


static void _enviar_caractere(uint8_t caractere) {
    _driver.enviar_dado(caractere);
}


static int8_t _dado(void) {
    return _driver.dado();
}

static int8_t _count(void)
{
  return _driver.count();	
}

static void _enviar_string(uint8_t *string) {
    uint16_t i=0;
    for(i=0;string[i]!='\0';i++)
        _driver.enviar_dado(string[i]);
}

static int8_t _dados_bytes(void) {
    return _driver.dados_bytes();
}


static void _enviar_decimal(uint8_t valor) {
  static uint8_t cenmil = 0, dezmil = 0, mil = 0, cen = 0, dez = 0, uni = 0;

  cenmil = valor / 100000;
  dezmil = (valor%100000) / 10000;
  mil = (valor % 10000) / 1000;
  cen = (valor%1000) / 100;
  dez = (valor%100)/10;
  uni = (valor%10);
  
  _driver.enviar_dado(cenmil + 48);
  _driver.enviar_dado(dezmil + 48);
  _driver.enviar_dado(mil + 48);
  _driver.enviar_dado(cen + 48);
  _driver.enviar_dado(dez + 48);
  _driver.enviar_dado(uni + 48);
} 


static void _enviar_hexadecimal(uint8_t valor) {   
  uint8_t i,temp;
  for (i=0; i<2; i++)
  {
      temp = (valor & 0xF0)>>4;
      if ( temp <= 9)
          _driver.enviar_dado('0' + temp);
      else
          _driver.enviar_dado('A' + temp - 10);
      valor = valor << 4;    
    }
}


struct UART uart = {
    .enviar_caractere = _enviar_caractere,
    .enviar_string = _enviar_string,
    .enviar_hexadecimal = _enviar_hexadecimal,
    .enviar_decimal = _enviar_decimal,
    .dado = _dado,
    .count = _count,
    .dados_bytes = _dados_bytes
};