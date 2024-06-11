#define PWM_CHANNEL_1 1
#define PWM_CHANNEL_2 2
#include "PWM.h"
#include <stdlib.h>

uint16_t vect_duty_cycle[101];

void setup_pwm_e_duty_cycle(void) 
{
  /* Conversao de PWM para duty */
  for(int16_t i = 0; i < 101; i++)
  {
    vect_duty_cycle[i] = i*(1023.0/100.0);
  }
}

void calc_pwm_limit(bits_field *motors)
{            
  // Impede que os valores do pwm_a e pwm_b ultrapassem o limite pre-definido
  static int16_t ExcessoB = 0, ExcessoA = 0;
  if (motors->PWM_right > 1023)
  {
    ExcessoB = (abs(motors->PWM_right) - 1023);
    motors->PWM_right = 1023;
    motors->PWM_left -= ExcessoB;
  }

  else if (motors->PWM_left > 1023)
  {
    ExcessoA = (abs(motors->PWM_left) - 1023);
    motors->PWM_left = 1023;
    motors->PWM_right -= ExcessoA;
  }

  if (motors->PWM_right < -1023)
  {
    ExcessoB = (abs(motors->PWM_right) - 1023);
    motors->PWM_right = 1023;
    motors->PWM_left += ExcessoB;
  }

  else if (motors->PWM_left < -1023)
  {
    ExcessoA = (abs(motors->PWM_left) - 1023);
    motors->PWM_left = 1023;
    motors->PWM_right += ExcessoA;
  }
}

uint8_t get_pwm(uint16_t duty_cycle)
{
    static uint16_t pwm = 0;
    if (duty_cycle > 100) duty_cycle = 100;
    pwm = vect_duty_cycle[duty_cycle];
    return pwm;
}

// uint8_t get_pwm_medio_motores(void)
// {
//     uint8_t duty_ch1 = get_pwm_duty_service(PWM_CHANNEL_1);
//     uint8_t duty_ch2 = get_pwm_duty_service(PWM_CHANNEL_2);
//     return ((duty_ch1 + duty_ch2) / 2);
// }

// void set_pwm_motor_direito(uint16_t duty_cicle) 
// {
//     pwm_set_duty_service(duty_cicle, PWM_CHANNEL_1);
// }

// void set_pwm_motor_esquerdo(uint16_t duty_cicle)
// {
//     pwm_set_duty_service(duty_cicle, PWM_CHANNEL_2);
// }