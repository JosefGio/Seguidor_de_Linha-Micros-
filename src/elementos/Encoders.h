#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    void (*contar_pulsos)(void);
    uint16_t (*obter_pulsos)(void);
} Encoder;

Encoder encoder_direito;
Encoder encoder_esquerdo;

typedef enum {
    CANAL_A, CANAL_B
} canal_encoder_t;

typedef struct {
    bool (*testar_canal)(canal_encoder_t canal);
} IDriverEncoder;

void EncoderEsquerdo_init_driver(IDriverEncoder driver);
void EncoderDireito_init_driver(IDriverEncoder driver);

//========================================================================================
typedef struct {
    int16_t angular;
    int16_t linear;
} velocidades_t;

struct EncodersAplicados {
    velocidades_t (*calcular_velocidades_do_robo)(void);
} encoders;

#endif