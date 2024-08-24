#include <avr/io.h>
#include "plataforma/drivers_setup.h"
#include "elementos/Motores.h"
#include "PWM.h"

#define REG_MOTORES        PORTD
#define REG_PWM_1          OCR1A
#define REG_PWM_2          OCR1B


#define MOTOR_DIREITO_CANAL_1    PD5
#define MOTOR_DIREITO_CANAL_2    PD6
#define MOTOR_ESQUERDO_CANAL_1   PD3
#define MOTOR_ESQUERDO_CANAL_2   PD4

static void _ligar_canal_1(motor_t motor) {
    if (motor == MOTOR_DIREITO)
        REG_MOTORES |= (1<<MOTOR_DIREITO_CANAL_1);
    else if (motor == MOTOR_ESQUERDO)
        REG_MOTORES |= (1<<MOTOR_ESQUERDO_CANAL_1);
}


static void _ligar_canal_2(motor_t motor) {
    if (motor == MOTOR_DIREITO)
        REG_MOTORES |= (1<<MOTOR_DIREITO_CANAL_2);
    else if (motor == MOTOR_ESQUERDO)
        REG_MOTORES |= (1<<MOTOR_ESQUERDO_CANAL_2);
}


static void _desligar_canal_1(motor_t motor) {
    if (motor == MOTOR_DIREITO)
        REG_MOTORES &= ~(1<<MOTOR_DIREITO_CANAL_1);
    else if (motor == MOTOR_ESQUERDO)
        REG_MOTORES &= ~(1<<MOTOR_ESQUERDO_CANAL_1);
}


static void _desligar_canal_2(motor_t motor) {
    if (motor == MOTOR_DIREITO)
        REG_MOTORES &= ~(1<<MOTOR_DIREITO_CANAL_2);
    else if (motor == MOTOR_ESQUERDO)
        REG_MOTORES &= ~(1<<MOTOR_ESQUERDO_CANAL_2);
}


static void _alterar_velocidade(motor_t motor, uint16_t PWM) {
    if (motor == MOTOR_DIREITO)
        REG_PWM_1 = PWM; //valores de 0 - 1023
    else if (motor == MOTOR_ESQUERDO)
        REG_PWM_2 = PWM;      
}


void setup_driver_Motores(void) {
    IDriverMotores driver = {
        .desligar_canal_1 = _desligar_canal_1,
        .desligar_canal_2 = _desligar_canal_2,
        .ligar_canal_1 = _ligar_canal_1,
        .ligar_canal_2 = _ligar_canal_2,
        .alterar_velocidade = _alterar_velocidade
    };

    Motores_init_driver(driver);
}