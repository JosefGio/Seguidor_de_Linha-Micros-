/*
 * File:   main.c
 * Author: Wolfbotz-IEEE-CEFET-RJ
 *
 * Created on 30 de Maio de 2021, 16:52
 */

/* Este codigo eh de um robo seguidor de linha da equipe Wolfbotz.
 * Aqui nos vemos o controle do robo bem como as tomadas de decisao de acordo com os padroes da pista
 */

//371mm de diâmetro
#define COLETA                   0
#define TELEMETRIA               1
#define NUM_SENSORS 7


enum ComandosDeOperacaoUART {
    TOMAR_TEMPO='a', MAPEAR='m', PARAR='s'
};

#include <avr/interrupt.h>
#include "bits_field.h"
#include "../include/main.h"
#include "plataforma/drivers_setup.h"
#include "elementos/Led.h"
#include "elementos/Encoders.h"
#include "elementos/SensoresFrontais.h"
#include "elementos/GerenciadorDeTrajeto.h"
#include "elementos/UART.h"
#include "rotinas/rotinas.h"
#include "IHM.h"
#include "Running.h"
#include "elementos/Motores.h"

#define  CLOCK_CPU   16000000UL

uint16_t  millisegundos = 0;     // variável de cronometragem
bool start_timing = false;

bool aciona_controlar_direcao = 0;

int8_t buffer[20];

velocidades_t velocidades;
//==============================================================================

/* Estes valores serao inseridos manualmente apos o mapeamento */
void enviar_tempo(void);
void setup_timer0_interrupt(void);
void setup_external_interrupt_service(void);
void adc_setup(void);
void uart_setup(uint32_t bps, uint8_t fast);
void setup_timer0_interrupt(void);

ISR(TIMER0_OVF_vect)
{
    TCNT0 = 8; // recarrega o Timer 0 para que a contagem seja 1ms novamente
    f_timers();   
}

// Funcao principal
int main(void) 
{
    setup();
    while (1) loop();
    return 0;    
}

// ======================================================================================================
// ===================================== RTOS primitivo =================================================
// ======================================================================================================
// ============================== Parte nao visivel ao usuario ==========================================
// ======================================================================================================


void f_timers (void) /* Funcao e chamada a cada 500us */ 
{
    led_offboard.alternar();
    // ATÉ DEFINIR ONDE VAI FICAR ESSE TRATAMENTO
    if (estado_da_operacao.iniciada())
        start_timing = true;
    else if (estado_da_operacao.finalizada()) {
        //envia_dado();
        enviar_tempo();
        return;
    }
    
    //if(start_running)
    //{
        if(max_timer1.compare < max_timer1.counter) 
            max_timer1.compare++;
        else
        {
            f_timer1();
            max_timer1.compare = 1;
        }

        if(max_timer2.compare < max_timer2.counter) 
            max_timer2.compare++;
        else
        {
            f_timer2();
            max_timer2.compare = 1;
        }
    //}

    if(max_timer3.compare < max_timer3.counter) 
        max_timer3.compare++;
    else
    {
        f_timer3();
        max_timer3.compare = 1;
    }
}

/* tempo = 500us */
void f_timer1(void) 
{   
    // led_offboard.ligar();
    verificar_progressao_de_pista();
    atualizar_leitura();
    // led_offboard.desligar();
}

/* tempo = 1ms */
void f_timer2(void)
{   
    // led_offboard.ligar();
    millis();   // funcao chamada a cada 1ms
    // led_offboard.desligar();
    //if(uart.count() > 0)  
    //{
    //    select_menu_IHM();
    //}
}

void f_timer3(void)//a cada 10ms -> sujeito a alteracao de acordo com a inércia dos motores
{
    // led_offboard.ligar();
    controlar_direcao();
    // led_offboard.desligar();
}

void setup_Hardware_service(void)
{
  MCUCR &= 0xef;      //habilita pull up quando configurado e desabilita algumas configuracoes previas do MCU

  DDRD  = 0x7A;     //PD0, PD2 e PD7 como entrada, demais como saida
  PORTD = 0x00;   //todas as saidas iniciam em 0 e entradas sem pull-up
  DDRB  = 0x06;     //PB1 e PB2 como saida e demais como entrada
  PORTB = 0x00;     
  DDRC  = 0x01;     //PC0 saida
  PORTC = 0x00;   //inicio saída em LOW e entrada sem pull up     
  TCCR1A = 0xA2; //Configura operacao em fast PWM, utilizando registradores OCR1x para comparacao
  TCCR1B = 0b00011001; //Fast mode com ICR1 sem prescaler
  ICR1 = 8000; //Delimita aye uma frequencia de 2kHz
  uart_setup(115200, 1); //Inicializa a comunicacao UART com 57.6kbps; 
  adc_setup(); //Inicializa o AD
  setup_timer0_interrupt(); //Inicializa o Timer0
  setup_external_interrupt_service(); //Inicializo as interrupcoes externas
}

//==============================================================================

// ADC
//==============================================================================
void adc_setup(void)
{  
    ADMUX = 0x40; //0100-0000   //Referencia no AVCC, deslocado a esquerda    
    ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADATE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRB = 0x00; // 0000-0000 //Modo livre
    DIDR0 = 0x3e;// 0011-1111   //Desabilita a entrada digital desses terminais

  //Atenacao. Segundo o datasheet quanto maior a velocidade,
}

//==============================================================================

// UART
//==============================================================================
/*Variaveis da UART*/

void uart_setup(uint32_t bps, uint8_t fast)
{
    
  if(fast)                                     //se fast verdadeiro, double speed
  {
	UCSR0A |= (1<<U2X0);                       //double speed 
	UBRR0 = (CLOCK_CPU/(8UL*bps)) - 1;               //cálculo do baud rate  	  
  }
  
  else                                         //senão, operação normal da USART
  {
	UCSR0A &= ~(1<<U2X0);                      //modo normal
	UBRR0 = (CLOCK_CPU/(16UL*bps)) - 1;              //cálculo do baud rate para o modo normal  
	  
  }
  
  UCSR0B |= (1<<RXCIE0) |                      //habilita interrupção para RX
            (1<<TXCIE0) |                      //habilita interrupção para TX
			(1<<RXEN0)  |                      //habilita recepção USART
			(1<<TXEN0);                        //habilita transmissão da USART
			
			
  UCSR0C = 0x06;                               //USART assíncrona, sem paridade, 8 bits de dados, 1 stop bit
}



void setup_timer0_interrupt(void)
{ 
  TCCR0B = 0x03; //TC0 com prescaler de 64
  TCNT0  = 8;          //1ms
  TIMSK0 = 0x01; //habilita a interrupcao do TC0
  /*tempo =65536 * Prescaler/Fosc = 65536 * 1024/16000000 = 4, 19s
    tempo = X_bit_timer * Prescaler/Fosc
    Valor inicial de contagem = 256 - tempo_desejado*Fosc/Prescaler = 256 - 0,001*16000000/1024 = 255
    Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler */
}

void setup_external_interrupt_service(void)
{
  PCICR  = 0x05; //habilito a interrupcao externa do canal PCINT0 e PCINT2
  PCMSK0 = 0x01; //habilito a porta PCINT0 como interrupcao externa do canal PCINT0
  PCMSK2 = 0x04;  //habilito a porta PCINT18 como interrupcao do canal PCINT2
}

// ======================================================================================================
// =========================== Funcoes nao viriaveis ao usuario =========================================
// ======================================================================================================
void setup() 
{   
    cli();
    setup_Hardware_service();
    setup_driver_SensoresLaterais();
    setup_driver_Led_offboard();
    setup_driver_Encoders();
    setup_driver_SensoresFrontais();
    setup_driver_Motores();
    setup_driver_UART();
    setup_variaveis();
    // adc_conversion_ch_service(2);
    //clear_IHM();
    //show_options();
    //select_menu_IHM();
    estado_da_operacao.modo = TOMADA_DE_TEMPO;
    sei();
}


void setup_variaveis()
{        
    max_timer1.counter    = 1,  //1ms
    max_timer2.counter    = 1, //1ms
    max_timer3.counter    = 40,  //40ms
    max_timer1.compare = 1,
    max_timer2.compare = 1,
    max_timer3.compare = 1,    
    setup_pwm_e_duty_cycle();

    //ALTERAR ESTA VARIAVEL
    variable.PWM_front_motors = 0;//get_pwm(20);//get_pwm(35);
}

void loop()
{
    /*if(aciona_controlar_direcao)
    {   
        led_offboard.alternar();
        controlar_direcao();
        aciona_controlar_direcao = 0;
    }*/   
} // loop vazio

void millis(void)
{
    if(start_timing)
        millisegundos++;
}

void enviar_tempo(void)
{
    static bool enviado = false;
    if (enviado)
        return;
    enviado = true;
    sprintf((char *)buffer, "%d\n", millisegundos);
    uart.enviar_string(buffer);
}

// ======================================================================================================
// ===================================== FINAL DO CODIGO ================================================
// ======================================================================================================