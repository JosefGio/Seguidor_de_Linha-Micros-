#ifndef DEVICES_H
#define DEVICES_H
/*  DEFINIÇÃO DOS DISPOSITIVOS, USADOS NA APLICAÇÃO, A SEREM MAPEADOS NA 
    IMPLEMENTAÇÃO DA HAL EXTERNA */
//==============================================================================
//definir os laterais e os frontais
typedef enum OutputDevicesIdentifiers {
  AIN1, AIN2, BIN1, BIN2, END_OUTPUT_DEVICES
} out_id;

#define NUM_OUTPUT_DEVICES END_OUTPUT_DEVICES
//==============================================================================
#endif