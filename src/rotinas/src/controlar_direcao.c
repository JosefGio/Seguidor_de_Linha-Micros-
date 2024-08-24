// MÓDULO PARA ESTRATÉGIA DE CONTROLE DE DIREÇÃO DO ROBÔ

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "PID.h"
#include "PWM.h"
#include "bits_field.h"
#include "rotinas/rotinas.h"
#include "elementos/GerenciadorDeTrajeto.h"
#include "elementos/SensoresFrontais.h"
#include "elementos/Encoders.h"
#include "elementos/Motores.h"
#include "elementos/UART.h"

static void calcular_variavel_de_controle_rotacional(void);
void atualizar_rotacao_dos_motores(void);
void fora_da_pista(void);

// #define PID_X   // Ao comentar o PID translacional eh desabilitado
#define ANGULAR 0
#define LINEAR 1
extern int8_t buffer[20];
extern void envia_dado(void);

float vp_angular = 0;          // variavel de processo
float erro_w = 0, setpoint = 0, u_w = 0;

#define LIMIT_ANGULAR_MOTOR_SPEED 0.8 //a definir - rad/s
#define LIMIT_LINEAR_MOTOR_SPEED 0.8 //a definir - m/s
#define MAX_PWM 8000
#define VEL_CURVA_LVL_02 0.95 * MAX_PWM 
#define VEL_CURVA_LVL_01 0.75 * MAX_PWM
#define VEL_CURVA_LVL_0 0.65 MAX_PWM 
#define VEL_CURVA_LVL_1 0.58 * MAX_PWM 
#define VEL_CURVA_LVL_2 0.45 * MAX_PWM
#define VEL_CURVA_LVL_3 0.28 * MAX_PWM
#define VEL_CURVA_LVL_35 0.25 * MAX_PWM
#define VEL_CURVA_LVL_4 0.25 * MAX_PWM
#define VEL_CURVA_LVL_5 0.23 * MAX_PWM


#define VEL_RETA_LVL_02 0.95 * MAX_PWM
#define VEL_RETA_LVL_01 0.9 * MAX_PWM

#define VEL_RETA_LVL_0 0.65 * MAX_PWM
#define VEL_RETA_LVL_1 0.55 * MAX_PWM 
#define VEL_RETA_LVL_2 0.39 * MAX_PWM
#define VEL_RETA_LVL_24 0.38 * MAX_PWM
#define VEL_RETA_LVL_3 0.347 * MAX_PWM
#define VEL_RETA_LVL_35 0.345 * MAX_PWM
#define VEL_RETA_LVL_4 0.28 * MAX_PWM
#define VEL_RETA_LVL_5 0.23 * MAX_PWM

uint16_t setpoint_x_PWM[NUMERO_DE_TRECHOS] = {
    //Pista RIW 2024
    0.2*MAX_PWM, 
    //começo da pista 
    VEL_RETA_LVL_4,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3, //Curvona antes do cruzamento
    //Cruzamento
    VEL_RETA_LVL_3,VEL_RETA_LVL_3,VEL_CURVA_LVL_3,VEL_RETA_LVL_3,VEL_CURVA_LVL_3,VEL_RETA_LVL_3,VEL_RETA_LVL_3, //Segunda passagem no cruzamento
    //Curva pós cruzamamento
    VEL_CURVA_LVL_3,VEL_RETA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3,VEL_CURVA_LVL_3, //Última curva antes de sequência de cruzamentos
    //Reta antes de sequência de cruzamentos
    VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, // Primeira curva
    //Pós primeira curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Segunda curva
    //Pós segunda curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Terceira curva
    //Pós terceira curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_3,VEL_CURVA_LVL_3, //Quarta curva
    //Pós quarta curva 
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2, //Início quadrado de curvas
    //Quadrado de curvas
    VEL_CURVA_LVL_3, VEL_RETA_LVL_2, VEL_CURVA_LVL_3,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Fim do quadrado de curvas
    //Começo sequência de cruzamentos
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, // Quinta curva
    //Pós quinta curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Sexta curva
    //Pós sexta curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Sétima curva
    //Pós sétima curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Oitava curva 
    //Pós oitava curva
    VEL_RETA_LVL_2, VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_RETA_LVL_2,VEL_CURVA_LVL_3, //Curva final
    //Última reta
    VEL_RETA_LVL_3

};

// float v_robot[NUMERO_DE_TRECHOS] =
// {
//     0.2, 0.3, 0.25, 0.25, 0.6
// }; //velocidade em m/s
//usar ao alterar a arquitetura de controle

static float posicao_media_sensores_frotais = 0;

extern bool out_of_line;

uint8_t _trecho;

extern velocidades_t velocidades;

void controlar_direcao(void)
{

    posicao_media_sensores_frotais = sensores_frontais.posicao_media();
    //fora_da_pista(); //comentar ao calibrar

    //if (out_of_line)
    //    return;
    calcular_variavel_de_controle_rotacional();
    atualizar_rotacao_dos_motores(); //comentar ao calibrar sensores
}

void fora_da_pista(void) // ocorrer se apos algumas tentativas ele não voltar para a linha
{
    static uint16_t contador = 0;

    if (out_of_line)
    {
        contador++; // determina o numero de tentativas
        if (contador > 100) // se nao achar a linha em 1s...
        {
            motores.frear();
            motores.alterar_velocidade(MOTOR_DIREITO, 0);
            motores.alterar_velocidade(MOTOR_ESQUERDO, 0);
            // envia_dado();
        }
    }

    else // voltei para a linha
    {
        contador = 0;
    }
}
static float error_MR = 0, errorML = 0, u_MR = 0, u_ML = 0;

static void calcular_variavel_de_controle_rotacional(void)
{
    static float VR = 0, VL = 0, w_ref_MR = 0, w_ref_ML = 0;

    // _trecho = trechos.trecho_atual();
    // setpoint = 0;
    // vp_angular = posicao_media_sensores_frotais;
    // erro_w = setpoint - vp_angular; // Variavel de processo (PV)
    // u_w = PID_angular(erro_w);    // Variavel manipulada  (MV)
    //o controle de posicao vai servir para fornecer a velocidade que o robo pecisa ter para gerar correcao emcima da linha

    // VR  = ((2*v_robot[_trecho]) + (u_w*LENGTH))/(2);
    // VL  = ((2*v_robot[_trecho]) - (u_w*LENGTH))/(2);

    //converting linear motor velocity to angular motor velocity
    // w_ref_MR = VR/RADIUS;
    // w_ref_ML = VL/RADIUS;

    //descomentar ao ter os valores maximos dos motores
    // if(w_ref_MR > LIMIT_ANGULAR_MOTOR_SPEED) //limit of motor angular frequency 
    //     w_ref_MR = LIMIT_ANGULAR_MOTOR_SPEED;
    // else if (w_ref_MR < -LIMIT_ANGULAR_MOTOR_SPEED)
    //     w_ref_MR = -LIMIT_ANGULAR_MOTOR_SPEED;

    // if(w_ref_ML > LIMIT_ANGULAR_MOTOR_SPEED)
    //     w_ref_ML = LIMIT_ANGULAR_MOTOR_SPEED;
    // else if (w_ref_ML < -LIMIT_ANGULAR_MOTOR_SPEED)
    //     w_ref_ML = -LIMIT_ANGULAR_MOTOR_SPEED;

    // if(w_ref_MR > 0 && w_ref_ML < 0)
    // {
    //     motores.manobrar_para_esquerda();
    //     w_ref_ML = fabs(w_ref_ML);
    // }
    // else if(w_ref_MR < 0 && w_ref_ML > 0)
    // {
    //     w_ref_MR = fabs(w_ref_MR);
    //     motores.manobrar_para_direita();
    // }

    // else
    // {
    //     motores.ir_para_frente();
    // }
    
    // error_MR = w_ref_MR - get_angular_freq_motorMR();
    // errorML = w_ref_ML - get_angular_freq_motorML();
    // u_MR = PID_angular(error_MR);
    // u_ML = PID_angular(errorML);

    // variable.PWM_right = u_MR*(MAX_PWM/12); //PWM/volts(tensão da bateria)
    // variable.PWM_left = u_ML*(MAX_PWM/12); //PWM/volts


    //párte do codigo semelhanta ao que ja era so que utiilizando o deslocamento do robo
    setpoint = posicao_media_sensores_frotais;
    vp_angular = dist_linear();

    erro_w = setpoint - vp_angular;

    u_w = PID_angular(erro_w);

    variable.PWM_right = setpoint_x_PWM[_trecho] + u_w; // Saida do controle translacional corrigindo o PWM em questão
    variable.PWM_left = setpoint_x_PWM[_trecho] - u_w;  // Saida do controle rotacional corrigindo o PWM em questão

    //  sprintf((char*)buffer, "%d\t%d\n", setpoint, vp_angular);
    //  uart.enviar_string(buffer);
}

void atualizar_rotacao_dos_motores(void)
{
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