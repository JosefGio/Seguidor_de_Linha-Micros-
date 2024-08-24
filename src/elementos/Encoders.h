#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include <stdbool.h>

#define PULSES 30  //resultado da relacao de engrenages do motor 5:1 - 5 pulsos * 6 dipolos mag
#define TS     0.010 //10 ms - alterado de acordo com a funcao que chama a controlar direcao
#define RADIUS 0.014 //14 mm
#define LENGTH 0.12 //120 mm
#define PULSE_TO_LINEAR_DIST 2*M_PI*RADIUS/PULSES //pulses/m

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

float get_angular_freq_motorMR(void);
float get_angular_freq_motorML(void);
float linear_speed(void);
float angular_speed(void);
float dist_linear(void);
float dist_angular(void);

struct EncodersAplicados {
    velocidades_t (*calcular_velocidades_do_robo)(void);
} encoders;

#endif