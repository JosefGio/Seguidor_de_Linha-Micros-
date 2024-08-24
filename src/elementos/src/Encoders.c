#include "elementos/Encoders.h"
#include "elementos/UART.h"
#include <math.h>

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

extern int8_t buffer[20];

uint16_t delta_EncR = 0;
uint16_t delta_EncL = 0;

float get_angular_freq_motorMR(void)
{
    static float w_raw = 0;
    static uint16_t pulso_atualR, pulso_anteriorlR;
    
    pulso_atualR = encoder_direito.obter_pulsos();
    delta_EncR = pulso_atualR - pulso_anteriorlR; //variacao de pulsos


    w_raw = (2.0 * 3.141592 * delta_EncR)/(PULSES*TS); 

    pulso_anteriorlR = pulso_atualR;
    return w_raw;
}

float get_angular_freq_motorML(void)
{
    static float w_raw = 0;
    static uint16_t pulso_atualL, pulso_anteriorlL;

    pulso_atualL = encoder_esquerdo.obter_pulsos();
    delta_EncL = pulso_atualL - pulso_anteriorlL; //variacao de pulsos
    
    w_raw = (2.0 * 3.141592 * delta_EncL)/(PULSES*TS); 

    pulso_anteriorlL = pulso_atualL;
    return w_raw;
}

float linear_speed(void)
{
    float w1 = get_angular_freq_motorMR();
    float w2 = get_angular_freq_motorML();

    return (((w1 + w2)*RADIUS)/2);
}

float angular_speed(void)
{
    float w1 = get_angular_freq_motorMR();
    float w2 = get_angular_freq_motorML();

    return (((w1 - w2)*RADIUS)/LENGTH);
}

float dist_linear(void)
{
    static uint16_t delta_s = 0;

    delta_s = ((delta_EncR+delta_EncL)*PULSE_TO_LINEAR_DIST)/2;

    return (delta_s);
}

float dist_angular(void)
{
    static float delta_theta;

    delta_theta = ((delta_EncR-delta_EncL)*PULSE_TO_LINEAR_DIST)/LENGTH;
    return (delta_theta);
}

//--------------------//

velocidades_t _calcular_velocidades_do_robo(void) 
{

    velocidades_t velocidades = {
        .angular = angular_speed(), // conversao em rad/s
        .linear = linear_speed()     // conversao em dm/s 
    };

    return velocidades;
}


struct EncodersAplicados encoders = {
    .calcular_velocidades_do_robo = _calcular_velocidades_do_robo
};

//========================================================================================