#include "rotinas/rotinas.h"
#include "elementos/SensoresLaterais.h"
#include "elementos/Led.h"
#include "elementos/MapeadorDeTrajeto.h"
#include "elementos/GerenciadorDeTrajeto.h"
#include "motores.h"
#include "elementos/Motores.h"

static void tratar_progressao_de_pista(int16_t sensor_lido);
static void atualizar_progressao_de_trecho(void);
static void atualizar_progressao_de_operacao(void);
static void iniciar_operacao(void);
static void finalizar_operacao(void);


void verificar_progressao_de_pista(void) {
    static int16_t sensoriamento_anterior = 0;
    int16_t sensoriamento_atual = sensores_laterais.obter_leitura();

    if (sensoriamento_atual == sensoriamento_anterior) 
        return;

    sensoriamento_anterior = sensoriamento_atual;
    tratar_progressao_de_pista(sensoriamento_atual);
}


void tratar_progressao_de_pista(int16_t sensor_lido) {
    // VERIFICA SE PROGREDIU EM TRECHO (VIA MARCADOR DE CURVA)
    // OU SE PROGREDIU EM OPERAÇÃO (VIA MARCADOR DE PARADA)
    
    static bool ainda_lendo_sensor_de_parada = false;

    if (sensor_lido == S_CURVA || sensor_lido == NENHUM) {
        ainda_lendo_sensor_de_parada = false;
        led_offboard.desligar();
        
        if (sensor_lido == S_CURVA)
            atualizar_progressao_de_trecho();
        return;
    }

    if (ainda_lendo_sensor_de_parada) 
        return;
    
    // CÓDIGOS ABAIXO: LENDO PELA PRIMEIRA VEZ SENSOR DE PARADA OU CRUZAMENTO
    ainda_lendo_sensor_de_parada = true;
    led_offboard.ligar();
    atualizar_progressao_de_operacao();
}


void atualizar_progressao_de_trecho(void) {
    mapeador.atualizar();
    trechos.atualizar();
}


void atualizar_progressao_de_operacao(void) {
    estado_da_operacao.atualizar();

    // ABAIXO: TRATAMENTO DA OPERAÇÃO

    if (estado_da_operacao.iniciada())
    {
        iniciar_operacao();
        trechos.atualizar();
    }
    else if (estado_da_operacao.finalizada())
        finalizar_operacao();
}


void iniciar_operacao(void) {
    if (estado_da_operacao.modo == MAPEAMENTO)
        mapeador.iniciar();
}


void finalizar_operacao(void) {
    //motores_freio();
    motores.frear();

    if (estado_da_operacao.modo == MAPEAMENTO)
        mapeador.finalizar();
}
