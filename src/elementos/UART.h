#ifndef UART_H
#define UART_H

#include <stdint.h>


struct UART {
    void (*enviar_caractere)(int8_t caractere);
    void (*enviar_string)(int8_t *string);
    void (*enviar_hexadecimal)(uint8_t valor);
    void (*enviar_decimal)(uint8_t valor);
    int8_t (*dado)(void);
    int8_t (*count)(void);
    int8_t (*dados_bytes)(void);
} uart;


typedef struct {
    void (*enviar_dado)(uint8_t dado);
    int8_t (*dado)(void);
    int8_t (*read)(void);
    int8_t (*count)(void);
    int8_t (*dados_bytes)(void);
} IDriverUART;


void UART_init_driver(IDriverUART driver);


#endif