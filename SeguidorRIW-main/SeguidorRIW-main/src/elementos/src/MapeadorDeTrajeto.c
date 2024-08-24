#include "elementos/MapeadorDeTrajeto.h"
#include "elementos/Led.h"
#include "elementos/Encoders.h"
#include "elementos/GerenciadorDeTrajeto.h"
#include "elementos/UART.h"


//VARIÁVEIS DOS ENCODERS
//===============================================================================
static uint16_t pulsos_direito[NUMERO_DE_TRECHOS];
static uint16_t pulsos_esquerdo[NUMERO_DE_TRECHOS];
static uint16_t pulsos_direito_anterior = 0;
static uint16_t pulsos_esquerdo_anterior = 0;
//===============================================================================

extern int8_t buffer[20];

static void mapear_pulsos_dos_encoders(void) {  
    uint8_t _trecho = trechos.trecho_atual();

    pulsos_direito[_trecho] = encoder_direito.obter_pulsos() - pulsos_direito_anterior;
    pulsos_direito_anterior = encoder_direito.obter_pulsos();
    
    pulsos_esquerdo[_trecho] = encoder_esquerdo.obter_pulsos() - pulsos_esquerdo_anterior;
    pulsos_esquerdo_anterior = encoder_esquerdo.obter_pulsos();
}

static void _atualizar(void) {
    // ABAIXO TERÃO FUNÇÕES DE MAPEAMENTO DE DADOS
    mapear_pulsos_dos_encoders();    
}

static void _iniciar(void) {
    pulsos_direito_anterior = encoder_direito.obter_pulsos();
    pulsos_esquerdo_anterior = encoder_esquerdo.obter_pulsos();
}

static void _finalizar(void) {    
    mapear_pulsos_dos_encoders();

    for(int i = 0; i < NUMERO_DE_TRECHOS; i++)
    {
        sprintf(buffer, "%d\t%d\n", pulsos_esquerdo[i], pulsos_direito[i]);
        uart.enviar_string(buffer);
    }
}


struct MapeadorDeTrajeto mapeador = {
    .iniciar = _iniciar,
    .finalizar = _finalizar,
    .atualizar = _atualizar
};