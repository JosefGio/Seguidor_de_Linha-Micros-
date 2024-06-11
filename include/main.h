#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>            //Bilioteca do C
#include <stdbool.h>          //Biblioteca que permite utilizar variavel booleana
#include <stdint.h>
#include <stdlib.h>
#include "PWM.h"              //Biblioteca de PWM fast mode de 10 bits
#include "motores.h"
#include "sensors.h"          //Logica utilizando os sensores
#include "controller.h"
#include "Running.h"
//#include "HAL.h"

/*Prototipo das funcoes*/
void setup();
void blynk_led();
void SW_read(void);
void setup_variaveis();        //variaveis utilizadas na logica
void loop();
void set_true_flag_uart_rx(void);
void tratar_pwm_de_operacao(void);
void tomada_de_tempo();     //estrategia de tomada de tempo
void mapeamento();          //estrategia de mapeamento
//---------------------------------------------------------------------//
void tratar_leitura_marcador_de_parada();              //Leitura dos sensores laterais
void tratar_fim_de_pista();        //verifica se e o fim da pista 
//---------------------------------------------------------------------//
void count_pulsesE();
void count_pulsesD();
void millis(void);
//---------------------------------------------------------------------//
void f_timers (void);       //funcao de temporizacao das rotinas
void f_timer1(void);
void f_timer2(void);
void f_timer3(void);
void f_timer4(void);
void f_timer5(void);
void f_timer6(void);
void sub_timer(void);
void testes_uart();
/*===========================================================================*/

/*Macros*/
#define NOP() __asm__ __volatile__ ("nop")

#endif