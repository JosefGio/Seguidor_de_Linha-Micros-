// #ifndef HAL_H
// #define HAL_H


// #include <stdint.h>
// /* int16_tERFACE PARA UMA HAL EXTERNA:
//  * 
//  * - ESTA int16_tERFACE DETERMINA O QUE DEVERÁ SER IMPLEMENTADO POR UMA HAL EXTERNA
//  * - ESTA int16_tERFACE ESTÁ EXPONDO OS REQUISITOS DA APLICAÇÃO
//  */

// // DEFINIÇÃO DOS PROTOCOLOS A SEREM IMPLEMENTADOS PELA HAL EXTERNA
// //==============================================================================
// void setup_Hardware_service(void);

// // int16_terrupções ----------------------------------------------------------------
// void enable_interrupts_service(void);
// void disable_interrupts_service(void);
// void encoder_D_pin_change_interrupt_service(void *routine);
// void encoder_E_pin_change_interrupt_service(void *routine);
// void uart_interrupt_service(void *routine);
// void one_hundred_microsec_interrupt_service(void *routine);
// void lateral_sensors_change_interrupt_service(void *routine);
// void adc_interrupt_service(void *routine);

// // USART -----------------------------------------------------------------------
// void uart_caractere_sending_service(uint8_t ch);
// void uart_string_sending_service(int8_t *s);
// void uart_hex_sending_service(uint8_t ch);
// void uart_decimal_sending_service(int16_t data);

// int8_t get_uart_data(void);

// // PWM -------------------------------------------------------------------------
// // void pwm_set_duty_service(uint16_t duty, uint8_t channel);
// // uint16_t get_pwm_duty_service(uint8_t channel);

// // // ADC -------------------------------------------------------------------------
// // void adc_conversion_ch_service(uint8_t channel);
// // uint8_t adc_read_service(void);
// // // GPIO ------------------------------------------------------------------------
// // void setup_output_devices_pin_map(void);
// // void set_outputs_service(out_id id);
// // void clear_outputs_service(out_id id);
// // void flip_outputs_service(out_id id);

// // // DELAY ----------------------------------------------------------------------- 
// // void delay_ms(int16_t milliseconds);
// //==============================================================================

// #endif