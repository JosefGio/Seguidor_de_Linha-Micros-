#ifndef PWM_H
#define PWM_H
#include <stdint.h>
#include "HAL.h"
#include "bits_field.h"


void calc_pwm_limit(bits_field *motors);
uint8_t get_pwm(uint16_t duty_cycle);
uint8_t get_pwm_medio_motores(void);
void set_pwm_motor_direito(uint16_t duty_cicle);
void set_pwm_motor_esquerdo(uint16_t duty_cicle);
void setup_pwm_e_duty_cycle(void);

#endif