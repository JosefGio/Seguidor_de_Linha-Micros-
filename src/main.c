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
#include "main.h"
#include "bits_field.h"
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

uint8_t valor_max_abs = 215;
uint8_t valor_min_abs = 11;
int8_t velocidade_filtrada_angular = 0, velocidade_filtrada_linear = 0;
bool aciona_controlar_direcao = 0;

int8_t velocidade_angular_telemetrizada [500] = {0};
int8_t velocidade_linear_telemetrizada [500] = {0};
int8_t buffer[20];
int16_t sensores_filtrados = 0;

velocidades_t velocidades;
//==============================================================================

/* Estes valores serao inseridos manualmente apos o mapeamento */
void enviar_tempo(void);
void setup_timer0_interrupt(void);
void setup_external_interrupt_service(void);
void adc_setup(void);
void uart_setup(uint32_t bps, uint8_t fast);
void setup_timer0_interrupt(void);
void setup_external_interrupt_service(void);
void setup_pwm_setFreq(int8_t option);
int16_t filtro_media_motores_linear(void);
int16_t filtro_media_motores_angular(void);
int16_t sensores_frontais_media_movel(void);
int16_t filtro_media_exponencial_angular(void);
int16_t filtro_media_exponencial_linear(void);
void speedometer(void);
void envia_dado(void);

ISR(TIMER0_OVF_vect)
{
    TCNT0 = 131; // recarrega o Timer 0 para que a contagem seja 500us novamente
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
    // led_offboard.alternar();
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

        if(max_timer4.compare < max_timer4.counter) 
            max_timer4.compare++;
        else
        {
            f_timer4();
            max_timer4.compare = 1;
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
    //led_offboard.ligar();
    verificar_progressao_de_pista();
    atualizar_leitura();
    //led_offboard.desligar();
    //sensores_filtrados = sensores_frontais_media_movel();
}

/* tempo = 10ms */
void f_timer2(void) //fora de uso
{   
    // led_offboard.ligar();
    // led_offboard.desligar();
}

/* tempo = 1ms */
void f_timer3(void)
{   
    //led_offboard.ligar();
    millis();   // funcao chamada a cada 1ms
    //led_offboard.desligar();
    //if(uart.count() > 0)  
    //{
    //    select_menu_IHM();
    //}
}

void f_timer4(void)//a cada 10ms -> sujeito a alteracao de acordo com a inércia dos motores
{
    // led_offboard.ligar();
    velocidades = encoders.calcular_velocidades_do_robo();
    velocidade_filtrada_linear = filtro_media_motores_linear();
    velocidade_filtrada_angular = filtro_media_motores_angular();
    // sprintf(buffer, "%d\t%d\n", velocidade_filtrada_linear, velocidade_filtrada_angular);
    // uart.enviar_string(buffer);
    controlar_direcao();
    //aciona_controlar_direcao = 1;

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
  TCCR1A = 0xA3; //Configura operacao em fast PWM, utilizando registradores OCR1x para comparacao (PWM 8 bits)
  setup_pwm_setFreq(10);//TCCR1B = 10; //Seleciona opcao para frequencia de 2kHz de PWM    
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
  //FADC = 1MHz
  //Tadc = 1/1MHz = 1us
  //Primeira Conversao = 25*1 = 25us
  //Demais conversoes = 14*1 = 14us
  //Tempo total do prieiro ciclo = (25*1) + (14*1*5) = 95us
  //Tempo das demais conversoes = 14*1*6 = 84us
  //Utilizando teoria de amostragem -> 168->190us
  
  ADMUX = 0x60; //0110-0000   //Refer�ncia no AVCC, deslocado a direita
  ADCSRA = 0xac; //1010-1100  //ADC habilitado, interrup��o do AD habilitado e prescaler de 16 - 1MHz
  ADCSRB = 0x00; // 0000-0000 //Modo livre
  DIDR0 = 0x3e;// 0011-1111   //Desabilita a entrada digital desses terminais

  //Aten��o. Segundo o datasheet quanto maior a velocidade,
  //menor a precis�o do AD, logo, utilizar 8bits em freq. elevadas
}

//==============================================================================


void setup_pwm_setFreq(int8_t option) 
{
    /*
    TABLE:
  	//no fast Mode
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          9      16    kHz
          10       2    kHz
          11     250     Hz
          12     62,5    Hz
          13     15,6    Hz 
     */
    TCCR1B = option;
	
    /*
    TABLE:
  	//no phase corret PWM Mode
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          9       8     kHz
          10       1     kHz
          11     125      Hz
          12     31,25    Hz
          13     7,8      Hz
     */
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
  TCCR0B = 0x03; //TC0 com prescaler de 8
  TCNT0  = 131;          //125us
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
    //clear_IHM();
    //show_options();
    //select_menu_IHM();
    estado_da_operacao.modo = TOMADA_DE_TEMPO;
    sei();
}


void setup_variaveis()
{        
    max_timer1.counter    = 1,  //500us
    max_timer2.counter    = 40, //10ms
    max_timer3.counter    = 2,  //1ms
    max_timer4.counter    = 4,    //2ms
    max_timer1.compare = 1,
    max_timer2.compare = 1,
    max_timer3.compare = 1,
    max_timer4.compare = 1;
    
    setup_pwm_e_duty_cycle();

    //ALTERAR ESTA VARIAVEL
    variable.PWM_front_motors = 0;//get_pwm(20);//get_pwm(35);
}

int16_t filtro_media_motores_linear(void)
{
    #define NUMERO_DE_AMOSTRAS 20
    static int16_t speed[NUMERO_DE_AMOSTRAS] = {0};
    int16_t acumulador = 0;

    for (int i = NUMERO_DE_AMOSTRAS-1; i > 0; i--)
        speed[i] = speed[i-1];

    speed[0] = velocidades.linear;

    for(int i = 0; i <  NUMERO_DE_AMOSTRAS; i++)
    {
        acumulador += speed[i];
    }

    return acumulador/NUMERO_DE_AMOSTRAS;
}

int16_t filtro_media_motores_angular(void)
{
    #define NUMERO_DE_AMOSTRAS 20
    static int16_t speed[NUMERO_DE_AMOSTRAS] = {0};
    int16_t acumulador = 0;

    for (int i = NUMERO_DE_AMOSTRAS - 1; i > 0; i--)
        speed[i] = speed[i - 1];

    speed[0] = velocidades.angular;

    for (int i = 0; i < NUMERO_DE_AMOSTRAS; i++)
    {
        acumulador += speed[i];
    }

    return acumulador / NUMERO_DE_AMOSTRAS;
}

int16_t sensores_frontais_media_movel(void)
{
    //#define NUMERO_DE_AMOSTRAS 40
    static int16_t line_sensor[NUMERO_DE_AMOSTRAS] = {0};
    int16_t acumulador = 0;

    for (int i = NUMERO_DE_AMOSTRAS - 1; i > 0; i--)
        line_sensor[i] = line_sensor[i - 1];

    line_sensor[0] = sensores_frontais.posicao_media();

    for (int i = 0; i < NUMERO_DE_AMOSTRAS; i++)
    {
        acumulador += line_sensor[i];
    }

    return acumulador / NUMERO_DE_AMOSTRAS;
}

int16_t filtro_media_exponencial_angular(void)
{
    static int16_t saida_atual = 0, saida_anterior = 0, entrada = 0;
    const float alpha = 0.15;

    entrada = velocidades.angular;

    saida_atual = alpha * entrada + (1-alpha) * saida_anterior;
    saida_anterior = saida_atual;
    return saida_atual;
}

int16_t filtro_media_exponencial_linear(void)
{
    static int16_t saida_atual = 0, saida_anterior = 0, entrada = 0;
    const float alpha = 0.15;

    entrada = velocidades.linear;

    saida_atual = alpha * entrada + (1-alpha) * saida_anterior;
    saida_anterior = saida_atual;
    return saida_atual;
}

void blynk_led()
{
    /*set_outputs_service(LED_OFFBOARD);
    delay_ms(500);
    clear_outputs_service(LED_OFFBOARD);
    delay_ms(500);
    set_outputs_service(LED_OFFBOARD);
    delay_ms(500);
    clear_outputs_service(LED_OFFBOARD);
    delay_ms(500);*/
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

//Criar uma funcao que armazene as amostras de velocidade angular e linear a cada 10 ms e ao termino da pista enviar os dados
void speedometer(void)
{
    static uint16_t i = 0;
   
    if(i < 499)
    {
        velocidade_angular_telemetrizada[i] = velocidade_filtrada_angular;
        velocidade_linear_telemetrizada[i] = velocidade_filtrada_linear;
        i++;
        //sprintf(buffer, "%d\t%d\n", velocidades.linear, velocidades_telemetrizadas[1][i]);
        //uart.enviar_string(buffer);
    } 
    else
       return;
}

void envia_dado(void)
{   
    static bool enviado = 0;

    if(!enviado)
    {
        for(uint16_t i = 0; i < 499; i++)
        {

            sprintf((char *)buffer, "\n%d\t%d", velocidade_angular_telemetrizada[i], velocidade_linear_telemetrizada[i]);
            uart.enviar_string(buffer);
        }
        uart.enviar_caractere('\n');
        enviado = 1;
    }

    else    
        return;
}

// ======================================================================================================
// ===================================== FINAL DO CODIGO ================================================
// ======================================================================================================