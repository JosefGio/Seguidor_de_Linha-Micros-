#include "elementos/GerenciadorDeTrajeto.h"

#define NUM_MARCADORES_PARADA 2
#define NUM_CRUZAMENTOS       52
#define NUM_MARCADORES_PARADA_TOTAIS (NUM_MARCADORES_PARADA + NUM_CRUZAMENTOS)
#define CURVA                  350

extern int16_t setpoint_x_PWM[NUMERO_DE_TRECHOS];
extern uint16_t pulso_atual, pulso_anterior;

// ESTADO DA OPERAÇÃO
//======================================================================================
static uint8_t _contagem_de_marcadores_de_parada = 0;

static void atualizar_operacao(void) {
    _contagem_de_marcadores_de_parada++;
}

static bool _iniciada(void) {
    return (_contagem_de_marcadores_de_parada == 1);
}

static bool _finalizada(void) {
    return (_contagem_de_marcadores_de_parada == NUM_MARCADORES_PARADA_TOTAIS);
}

struct EstadoDaOperacao estado_da_operacao = {
    .atualizar = atualizar_operacao,
    .finalizada = _finalizada,
    .iniciada = _iniciada,
    .modo = TOMADA_DE_TEMPO    
};
//======================================================================================


// TRECHOS
//======================================================================================
static uint8_t _trechos = 0;

static void atualizar_trecho(void) {
    //if(setpoint_x_PWM[_trechos] == CURVA){
    //    return;
    //}
    //else
        _trechos++;
    if(_trechos > NUMERO_DE_TRECHOS)
        _trechos = 0;
    //pulso_anterior = pulso_atual;   //atualizo para o segmento atual
    //pulso_atual = 0;                //zero
}

static uint8_t _trecho_atual(void) {
    if (_trechos >= NUMERO_DE_TRECHOS)
        return 0;

    return _trechos;
}

struct Trechos trechos = {
    .atualizar = atualizar_trecho,
    .trecho_atual = _trecho_atual
};
//======================================================================================