/* MÓDULO PARA GERENCIAR INFORMAÇÕES SOBRE A PROGRESSÃO DA PISTA DURANTE O TRAJETO

trechos:
- utilizada para mapear informações e determinar qual setpoint será usado

operação:
- utilizada para determinar continuidade ou não das rotinas
*/

#ifndef PROGRESSAO_DA_PISTA_H
#define PROGRESSAO_DA_PISTA_H

#define MARCADORES_CURVA 24 //varia por pista
#define NUMERO_DE_TRECHOS (MARCADORES_CURVA + 1)

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TOMADA_DE_TEMPO, MAPEAMENTO
} modo_de_operacao_t;


struct EstadoDaOperacao {
    void (*atualizar)(void);
    bool (*iniciada)(void);
    bool (*finalizada)(void);
    modo_de_operacao_t modo;
} estado_da_operacao;


struct Trechos {
    void(*atualizar)(void);
    uint8_t (*trecho_atual)(void);
} trechos;

#endif