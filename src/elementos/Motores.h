#ifndef _MOTORES_H
#define _MOTORES_H

#include <stdint.h>

typedef enum {
    MOTOR_DIREITO=1, MOTOR_ESQUERDO
} motor_t;


struct Motores {
    void (*ir_para_frente)(void);
    void (*manobrar_para_direita)(void);
    void (*manobrar_para_esquerda)(void);
    void (*frear)(void);
    void (*alterar_velocidade)(motor_t motor, uint16_t PWM);
} motores;


typedef struct {    
    void (*ligar_canal_1)(motor_t motor); // CANAL 1: MOTOR PARA FRENTE
    void (*ligar_canal_2)(motor_t motor); // CANAL 2: MOTOR PARA TRÁS
    void (*desligar_canal_1)(motor_t motor);
    void (*desligar_canal_2)(motor_t motor);
    void (*alterar_velocidade)(motor_t, uint16_t PWM); // vel. baseada em PWM (baixo nível)
} IDriverMotores;


void Motores_init_driver(IDriverMotores driver);

#endif