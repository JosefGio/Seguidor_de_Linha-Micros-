#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "elementos/Encoders.h"
#include "plataforma/drivers_setup.h"

#define ENCODER_DIREITO    PIND
#define ENCODER_ESQUERDO   PINB

#define ENCODER_DIREITO_CANAL_A    PD2
#define ENCODER_DIREITO_CANAL_B    PD7
#define ENCODER_ESQUERDO_CANAL_A   PB0
#define ENCODER_ESQUERDO_CANAL_B   PB5

static bool testar_canal_enc_direito(canal_encoder_t canal) {
    if (canal == CANAL_A)
        return (ENCODER_DIREITO & (1<<ENCODER_DIREITO_CANAL_A));
    else //if (canal == CANAL_B)
        return (ENCODER_DIREITO & (1<<ENCODER_DIREITO_CANAL_B));
}

static bool testar_canal_enc_esquerdo(canal_encoder_t canal) {
    if (canal == CANAL_A)
        return (ENCODER_ESQUERDO & (1<<ENCODER_ESQUERDO_CANAL_A));
    else //if (canal == CANAL_B)
        return (ENCODER_ESQUERDO & (1<<ENCODER_ESQUERDO_CANAL_B));
}

void setup_driver_Encoders(void) {
    IDriverEncoder driver_enc_direito = {
        .testar_canal = testar_canal_enc_direito
    };
    IDriverEncoder driver_enc_esquerdo = {
        .testar_canal = testar_canal_enc_esquerdo
    };

    EncoderEsquerdo_init_driver(driver_enc_esquerdo);
    EncoderDireito_init_driver(driver_enc_direito);
}

// AS MACROS DESSAS INTERRUPÇÕES ESTÃO AQUI POIS APENAS OS ENCODERS ESTÃO UTILIZANDO-AS.
// CASO HAJA A NECESSIDADE DAS INTERRUPÇÕES SERVIREM A OUTROS ELEMENTOS E/OU CASOS DE USO
// ENTÃO A SUGESTÃO É QUE FIQUEM NA main.c

ISR(PCINT2_vect) {
    encoder_direito.contar_pulsos();
}

ISR(PCINT0_vect) {
    encoder_esquerdo.contar_pulsos();
}