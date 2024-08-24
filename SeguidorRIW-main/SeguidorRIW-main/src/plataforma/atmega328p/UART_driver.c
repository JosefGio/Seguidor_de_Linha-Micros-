#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "plataforma/drivers_setup.h"
#include "elementos/UART.h"

#define SIZEBUFF    20
volatile uint8_t usart_run = 1;                //1 para usart rodando, 0 para usart ocupada
volatile char rx_buffer[SIZEBUFF] = {0x00};    //buffer para recepção serial
volatile uint16_t rx_buff = 0;                 //contador de bytes
uint8_t byte_sent = 0;

ISR(USART_RX_vect) 
{
    static uint16_t cnt_buff = 0;                //contador local de endereço para o buffer rx
  
    //if(byte_sent)  //envio somente de caractere
    //{
    //    cnt_buff = 0;
    //    byte_sent = 0;
    //}
    rx_buffer[cnt_buff] = UDR0;                  //armazena o byte recebido no endereço atual do buffer de rx
    rx_buff++;                                   //incrementa o contador global de bytes recebidos
    cnt_buff++;                                  //incrementa o contador local de endereços para o buffer de rx
    
    if(cnt_buff>=SIZEBUFF)
        cnt_buff = 0;
}

ISR(USART_TX_vect)
{
  usart_run = 1;                               //ocorrência da transmissão	
}


static void _enviar_dado(uint8_t dado) {
    UDR0 = dado;
    //Aguarda o buffer ser desocupado
    while (!(UCSR0A & (1<<UDRE0)));                 //com polling

    //while(!usart_run);                           //aguarda usart liberar  //por interrupção
    //usart_run = 0;                               //ocupa usart
    //UDR0 = dado;
}


static int8_t _dado(void) {
    //byte_sent = 1;
    //return rx_buffer[0];
    static uint16_t buff_local = 0;
    char byte_rx = 0;
  
    byte_rx = rx_buffer[buff_local];
    buff_local++;
    rx_buff--;
    
    if(buff_local>=SIZEBUFF)
      buff_local = 0;
    
    return byte_rx;
}


static int8_t _dados_bytes(void) {
    static uint16_t buff_local = 0;
    char byte_rx = 0;
  
    byte_rx = rx_buffer[buff_local];
    buff_local++;
    rx_buff--;
    
    if(buff_local>=SIZEBUFF)
      buff_local = 0;
    
    return byte_rx;
}

static int8_t _count(void)
{   
    //if(byte_sent)  
    //    return 0;
    return rx_buff;
}

void setup_driver_UART(void) {
    IDriverUART driver = {
        .enviar_dado = _enviar_dado,
        .dado = _dado,
        .count = _count,
        .dados_bytes = _dados_bytes
    };

    UART_init_driver(driver);
}