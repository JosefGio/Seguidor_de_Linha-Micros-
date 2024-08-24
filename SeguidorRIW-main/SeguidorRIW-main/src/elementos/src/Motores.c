#include "elementos/Motores.h"
#include <util/delay.h>

static IDriverMotores _driver;


void Motores_init_driver(IDriverMotores driver) {
    _driver = driver;
}


static void _ir_para_frente(void) {
    _driver.ligar_canal_1(MOTOR_DIREITO);
    _driver.ligar_canal_1(MOTOR_ESQUERDO);
    
    _driver.desligar_canal_2(MOTOR_DIREITO);
    _driver.desligar_canal_2(MOTOR_ESQUERDO);
}


static void _manobrar_para_direita(void) {
    _driver.ligar_canal_1(MOTOR_ESQUERDO);
    _driver.ligar_canal_2(MOTOR_DIREITO);

    _driver.desligar_canal_2(MOTOR_ESQUERDO);
    _driver.desligar_canal_1(MOTOR_DIREITO);
}


static void _manobrar_para_esquerda(void) {
    _driver.ligar_canal_1(MOTOR_DIREITO);
    _driver.ligar_canal_2(MOTOR_ESQUERDO);

    _driver.desligar_canal_2(MOTOR_DIREITO);
    _driver.desligar_canal_1(MOTOR_ESQUERDO);
}


static void _frear(void) {
    _delay_ms(150);
    _driver.desligar_canal_1(MOTOR_DIREITO);
    _driver.ligar_canal_2(MOTOR_DIREITO);
    _driver.desligar_canal_1(MOTOR_ESQUERDO);
    _driver.ligar_canal_2(MOTOR_ESQUERDO);
    _driver.alterar_velocidade(MOTOR_DIREITO, 300);
    _driver.alterar_velocidade(MOTOR_ESQUERDO, 300);
    _delay_ms(100);

    _driver.desligar_canal_1(MOTOR_DIREITO);
    _driver.desligar_canal_2(MOTOR_DIREITO);
    _driver.desligar_canal_1(MOTOR_ESQUERDO);
    _driver.desligar_canal_2(MOTOR_ESQUERDO);

    _driver.alterar_velocidade(MOTOR_DIREITO, 0);
    _driver.alterar_velocidade(MOTOR_ESQUERDO, 0);

}


static void _alterar_velocidade(motor_t motor, uint16_t PWM) {
    _driver.alterar_velocidade(motor, PWM);
}


struct Motores motores = {
    .ir_para_frente = _ir_para_frente,
    .manobrar_para_direita = _manobrar_para_direita,
    .manobrar_para_esquerda = _manobrar_para_esquerda,
    .frear = _frear,
    .alterar_velocidade = _alterar_velocidade
};