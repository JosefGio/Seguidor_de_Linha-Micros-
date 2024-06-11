#include "elementos/Encoders.h"
#include "elementos/UART.h"

#define LARGURA_DO_ROBO 1.20 //dm
#define L_ROBO LARGURA_DO_ROBO/2 //dm
#define RAIO_RODA    14.5 //mm

// ENCODER ESQUERDO
//========================================================================================
IDriverEncoder driver_enc_esquerdo;

void EncoderEsquerdo_init_driver(IDriverEncoder driver) {
    driver_enc_esquerdo = driver;
}

static uint16_t pulsos_encoder_esquerdo = 0;

static uint16_t obter_pulsos_encoder_esquerdo(void) {
    return pulsos_encoder_esquerdo;
}

static void contar_pulsos_encoder_esquerdo(void) {
    static bool canal_A_anterior = false;
    static bool canal_A_atual = false, canal_B_atual = false;
    
    canal_A_atual = driver_enc_esquerdo.testar_canal(CANAL_A);
    if (canal_A_atual != canal_A_anterior) {
      canal_B_atual = driver_enc_esquerdo.testar_canal(CANAL_B);
      if (canal_B_atual != canal_A_atual)
        pulsos_encoder_esquerdo--;
      else
        pulsos_encoder_esquerdo++;  // sentido horario
    }
    
    canal_A_anterior = canal_A_atual;
}

Encoder encoder_esquerdo = {
    .contar_pulsos = contar_pulsos_encoder_esquerdo,
    .obter_pulsos = obter_pulsos_encoder_esquerdo
};
//========================================================================================


// ENCODER DIREITO
//========================================================================================
IDriverEncoder driver_enc_direito;

void EncoderDireito_init_driver(IDriverEncoder driver) {
    driver_enc_direito = driver;
}

static uint16_t pulsos_encoder_direito = 0;

static uint16_t obter_pulsos_encoder_direito(void) {
    return pulsos_encoder_direito;
}

static void contar_pulsos_encoder_direito(void) {
    static bool canal_A_anterior = false;
    static bool canal_A_atual = false, canal_B_atual = false;
    
    canal_A_atual = driver_enc_direito.testar_canal(CANAL_A);
    if (canal_A_atual != canal_A_anterior) {
      canal_B_atual = driver_enc_direito.testar_canal(CANAL_B);
      if (canal_B_atual != canal_A_atual) 
        pulsos_encoder_direito++;
      else
        pulsos_encoder_direito--;  // sentido anti-horario
    }
    
    canal_A_anterior = canal_A_atual;
}

Encoder encoder_direito = {
    .contar_pulsos = contar_pulsos_encoder_direito,
    .obter_pulsos = obter_pulsos_encoder_direito
};
//========================================================================================

// ENCODERS
//========================================================================================

//const float conversao = 1.765;
const float conversao = 1.54;   //rotacao/pulsos -> 29*pi/60pulsos
const float conv_segundos = 500.0;  //quando a base de tempo é de 2ms

extern int8_t buffer[20];

static int16_t calcular_velocidade_roda_direita(void) {
    static uint16_t pulso_atualR, pulso_anteriorlR;
    static uint16_t deltaR = 0, delta_EncR = 0;
    uint16_t vel = 0;
    
    pulso_atualR = encoder_direito.obter_pulsos();
    delta_EncR = pulso_atualR - pulso_anteriorlR; //variacao de pulsos
    deltaR = delta_EncR * conversao;  //distancia em mm (variacao da distância)

    if(!deltaR) 
        vel = 0;  //roda parada
    else
    {
        vel = deltaR * conv_segundos;  //converte para mm/s
        pulso_anteriorlR = pulso_atualR;
        pulso_atualR = 0x00;    //0
    }

    //sprintf(buffer, "%d\n", (int)(vel));
    //uart.enviar_string(buffer);

    return vel;
}


static int16_t calcular_velocidade_roda_esquerda(void) {
    static uint16_t pulso_atualL, pulso_anteriorlL;
    static uint16_t deltaL = 0;
    static uint16_t delta_EncL = 0;
    uint16_t vel = 0;

    pulso_atualL = encoder_esquerdo.obter_pulsos();
    delta_EncL = pulso_atualL - pulso_anteriorlL; //variacao de pulsos
    deltaL = delta_EncL * conversao;  //distancia em dm (variacao da distância)

    if(!deltaL) 
        vel = 0; //roda parada
    else
    {
        vel = deltaL * conv_segundos;  //variacao da distancia * fator de conversão para mm/s
        pulso_anteriorlL = pulso_atualL;
        pulso_atualL = 0x00;
    }

    //sprintf(buffer, "%d\n", (int)(vel));
    //uart.enviar_string(buffer);

    return vel;
}


velocidades_t _calcular_velocidades_do_robo(void) {
    int16_t v_roda_direita = calcular_velocidade_roda_direita();
    int16_t v_roda_esquerda = calcular_velocidade_roda_esquerda();

    velocidades_t velocidades = {
        //.angular = ((v_roda_direita - v_roda_esquerda) * RAIO_RODA) / (2 * LARGURA_DO_ROBO) * (180/3.14), //rad/s
        // .angular = ((v_roda_direita - v_roda_esquerda)) / (2 * 100 * L_ROBO), // conversao em rad/s
        .angular = ((v_roda_direita - v_roda_esquerda)) / (2 * 100 * LARGURA_DO_ROBO), // conversao em rad/s
        .linear = (v_roda_direita + v_roda_esquerda) / (2 * 100)     // conversao em dm/s 
    };

    return velocidades;
}


struct EncodersAplicados encoders = {
    .calcular_velocidades_do_robo = _calcular_velocidades_do_robo
};

//========================================================================================