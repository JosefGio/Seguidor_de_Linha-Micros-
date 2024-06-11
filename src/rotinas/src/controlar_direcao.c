// MÓDULO PARA ESTRATÉGIA DE CONTROLE DE DIREÇÃO DO ROBÔ

#include <stdbool.h>
#include <stdint.h>
#include "PID.h"
#include "PWM.h"
#include "bits_field.h"
#include "rotinas/rotinas.h"
#include "elementos/GerenciadorDeTrajeto.h"
#include "elementos/SensoresFrontais.h"
#include "elementos/Encoders.h"
#include "elementos/Motores.h"
#include "elementos/UART.h"

static void calcular_variavel_de_controle_translacional(void);
static void calcular_variavel_de_controle_rotacional(void);
void atualizar_rotacao_dos_motores(void);
void fora_da_pista(void);

// #define PID_X   // Ao comentar o PID translacional eh desabilitado
#define ANGULAR 0
#define LINEAR 1
extern int8_t buffer[20];
extern uint8_t velocidade_filtrada_linear;
extern int8_t velocidade_filtrada_angular;
extern void envia_dado(void);

int16_t pwm_motores_general = 0; // variável temporária para troca de valores
int16_t vp_angular = 0;          // variavel de processo
int16_t u_x = 0, u_w = 0;
uint16_t speed_x;
int16_t PV_linear = 0, erro_x = 0, erro_w = 0;
extern int16_t sensores_filtrados;

#define VEL_CURVA_LVL_1 250
#define VEL_CURVA_LVL_2 240
#define VEL_CURVA_LVL_3 220
#define VEL_CURVA_LVL_4 200

#define VEL_RETA_LVL_1 540
#define VEL_RETA_LVL_2 450
#define VEL_RETA_LVL_3 300
#define VEL_RETA_LVL_4 250
#define VEL_RETA_LVL_5 180

int16_t setpoint_x_PWM[NUMERO_DE_TRECHOS] = {
    VEL_RETA_LVL_5, VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_RETA_LVL_3, VEL_RETA_LVL_3, VEL_CURVA_LVL_1,
    VEL_RETA_LVL_3, VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_RETA_LVL_3, VEL_RETA_LVL_3, VEL_CURVA_LVL_1,
    VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_CURVA_LVL_1, VEL_RETA_LVL_3, VEL_RETA_LVL_3, VEL_RETA_LVL_3,
    VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_RETA_LVL_3, VEL_CURVA_LVL_1, VEL_RETA_LVL_3};

/*int16_t velocidade_mm_s[NUMERO_DE_TRECHOS] = {
250, 450, 350, 450, 450, 350,
450, 450, 350, 450, 450, 350,
450, 350, 350, 450, 450, 450,
450, 350, 450, 350, 450, 350, 450};*/

int16_t PV_angular = 0, setpoint = 0;

static int16_t posicao_media_sensores_frotais = 0;

extern bool out_of_line;

uint8_t _trecho;

extern velocidades_t velocidades;
void controlar_direcao(void)
{

    posicao_media_sensores_frotais = sensores_frontais.posicao_media();
    fora_da_pista(); //comentar ao calibrar

    if (out_of_line)
        return;
    calcular_variavel_de_controle_translacional();
    calcular_variavel_de_controle_rotacional();

    atualizar_rotacao_dos_motores(); //comentar ao calibrar sensores
}

void fora_da_pista(void) // ocorrer se apos algumas tentativas ele não voltar para a linha
{
    static uint16_t contador = 0;
    static int16_t memoria_posicao_media_sensores_frotais = 0;
    static uint8_t att_de_correcao = 0, first_stack = 1, first_correction = 1;

    if (out_of_line)
    {
        contador++; // determina o numero de tentativas
        // att_de_correcao++;
        // // parar o motor
        // if (first_correction)
        // {
        //     motores.alterar_velocidade(MOTOR_DIREITO, 0);
        //     motores.alterar_velocidade(MOTOR_ESQUERDO, 0);
        //     first_correction = 0;
        //     return;
        // }
        // // verificar o ultimo sensor lido e gira para o sentido lido
        // if (memoria_posicao_media_sensores_frotais < 0)
        // {
        //     motores.manobrar_para_direita();
        //     motores.alterar_velocidade(MOTOR_DIREITO, 150);
        //     motores.alterar_velocidade(MOTOR_ESQUERDO, 150);
        //     // rodas no mesmo sentido de giro
        //     // motores.frente();
        //     //  motores.alterar_velocidade(MOTOR_DIREITO, 50);
        //     //  motores.alterar_velocidade(MOTOR_ESQUERDO, 100);
        // }
        // else if (memoria_posicao_media_sensores_frotais > 0)
        // {
        //     motores.manobrar_para_esquerda();
        //     motores.alterar_velocidade(MOTOR_DIREITO, 150);
        //     motores.alterar_velocidade(MOTOR_ESQUERDO, 150);
        //     // rodas no mesmo sentido de giro
        //     // motores.frente();
        //     //  motores.alterar_velocidade(MOTOR_DIREITO, 100);
        //     //  motores.alterar_velocidade(MOTOR_ESQUERDO, 50);
        // }

        if (contador > 100) // se nao achar a linha em 1s...
        {
            motores.frear();
            // envia_dado();
            while (1) // prendo meu código
            {
                motores.alterar_velocidade(MOTOR_DIREITO, 0);
                motores.alterar_velocidade(MOTOR_ESQUERDO, 0);
                // envia_dado();
            }
        }
    }

    else // voltei para a linha
    {
        first_correction = 1;
        first_stack = 1;
        att_de_correcao = 0;
        contador = 0;
    }

    if (first_stack)
    {
        memoria_posicao_media_sensores_frotais = posicao_media_sensores_frotais;
        first_stack = 0;
    }

    if (att_de_correcao > 10) // a cada 100ms atualizo o valor anterior
    {
        memoria_posicao_media_sensores_frotais = posicao_media_sensores_frotais;
        att_de_correcao = 0;
    }
}

static void calcular_variavel_de_controle_translacional(void)
{
#define PID_X // comentado indica em malha aberta
    _trecho = trechos.trecho_atual();

    // variable.PWM_front_motors = setpoint_x_PWM[_trecho];

    speed_x = (0.035 * setpoint_x_PWM[_trecho]) - 1.51; // 10 bits-> dm/s
// speed_x  = (12.07  * setpoint_x_PWM[_trecho])  - 242.0; //8 bits mode
#ifdef PID_X
    PV_linear = velocidade_filtrada_linear;
    erro_x = speed_x - PV_linear; // Variavel de processo (PV)
    u_x = PID(erro_x, ENCODERS);  // Variavel manipulada  (MV)
#endif
    // sprintf(buffer, "%d\t%d\t%d\n", speed_x, PV_linear, u_x);
    // uart.enviar_string(buffer);
}

static void calcular_variavel_de_controle_rotacional(void)
{
// #define OPEN_LOOP
#define CLOSED_LOOP

#ifdef CLOSED_LOOP
    setpoint = posicao_media_sensores_frotais;
    vp_angular = velocidade_filtrada_angular;
    erro_w = setpoint - vp_angular; // Variavel de processo (PV)
    u_w = PID(erro_w, FRONTAIS);    // Variavel manipulada  (MV)
#endif

#ifdef OPEN_LOOP
    setpoint = 0;
    vp_angular = posicao_media_sensores_frotais;
    erro_w = setpoint - vp_angular; // Variavel de processo (PV)
    u_w = PID(erro_w, FRONTAIS);    // Variavel manipulada  (MV)
#endif
    // sprintf(buffer, "%d\t%d\n", setpoint, vp_angular);
    // uart.enviar_string(buffer);
}

void atualizar_rotacao_dos_motores(void)
{
    motores.ir_para_frente();

    variable.PWM_right = variable.PWM_front_motors + u_w + u_x; // Saida do controle translacional corrigindo o PWM em questão
    variable.PWM_left = variable.PWM_front_motors - u_w + u_x;  // Saida do controle rotacional corrigindo o PWM em questão
    // sprintf(buffer, "Antes: %d\t%d\n", variable.PWM_right, variable.PWM_left);
    // uart.enviar_string(buffer);
    calc_pwm_limit(&variable);

    if (variable.PWM_right < 0)
    {
        motores.manobrar_para_direita();
    }

    if (variable.PWM_left < 0)
    {
        motores.manobrar_para_esquerda();
    }

    motores.alterar_velocidade(MOTOR_DIREITO, variable.PWM_right);
    motores.alterar_velocidade(MOTOR_ESQUERDO, variable.PWM_left);
}