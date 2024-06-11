#include "IHM.h"

#define MENU_MAX 9

uint8_t menu = 0;
uint8_t keyb = 0;

void proportional_Linear_menu(void);
void integral_Linear_menu(void);
void derivative_Linear_menu(void);
void proportional_angular_menu(void);
void integral_angular_menu(void);
void derivative_angular_menu(void);

void clear_IHM_menu(void)
{
    for (uint16_t i = 0; i < 400; i++){
        uart.enviar_caractere('\n');
    }

    menu = 0;
}

void clear_IHM(void)
{
    for (uint16_t i = 0; i < 400; i++){
        uart.enviar_caractere('\n');
    }    
}

void show_options(void)
{
    uart.enviar_string("Menu de selecao. Selecione qual opcao configurar:\n");
    uart.enviar_string("a - Proximo Menu\n");
    uart.enviar_string("b - Menu Anterior\n");
    uart.enviar_string("c - Limpar Tela\n");
    uart.enviar_string("h - Menu de Ajuda\n");
}

void select_menu_IHM(void)
{   
    keyb = uart.dado();

    switch (keyb)
    {
        case 'a':
            menu++;
            if(menu>MENU_MAX) menu=MENU_MAX;
            break;
        
        case 'b':
            menu--;
            if(menu < 0) menu = 0;    
            break;
        
        case 'c':
            clear_IHM_menu();
            break;

        case 'h':
            show_options();
            break;
    }

    switch (menu)
    {
        case 1:
            menu_1();
            break;

        case 2:
            menu_2();
            break;

        case 3:
            menu_3();
            break;

        case 4:
            menu_4();
            break;

        case 5:
            menu_5();
            break;

        case 6:
            menu_6();
            break;

        case 7:
            menu_7();
            break;

        case 8:
            menu_8();
            break;
    }
}

void menu_1(void)
{
    proportional_angular_menu();
}

void menu_2(void)
{
    integral_angular_menu();
}

void menu_3(void)
{   
    derivative_angular_menu();
}

void menu_4(void){
    proportional_Linear_menu();
}

void menu_5(void){
    integral_Linear_menu();
}

void menu_6(void){
    derivative_Linear_menu();
}


void proportional_angular_menu(void){   
    extern float  kp_ang;
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 1----\nkp_ang: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*')  //similar a um comando de enter
    {
        string[i] = keyb;    //armazena o numero desejado
        i++;
    }
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        kp_ang = (float)atof(&string);
        i = 0;
    }
}

void integral_angular_menu(void)
{
    extern float ki_ang;
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 2----\nki_ang: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*'){
        string[i] = keyb;
        i++;
    }
    
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        ki_ang = atof(&string);
        i = 0;
    }
}

void derivative_angular_menu(void)
{
    extern float kd_ang;
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 3----\nkd_ang: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*'){
        string[i] = keyb;
        i++;
    }
    
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        kd_ang = atof(&string);
        i = 0;
    }    
}

void proportional_Linear_menu(void){
    extern float kp_line;
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 4----\nkp_lin: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*'){
        string[i] = keyb;
        i++;
    }
    
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        kp_line = atof(&string);
        i = 0;
    }
}

void integral_Linear_menu(void)
{
    extern float ki_line; 
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 5----\nki_lin: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*'){
        string[i] = keyb;
        i++;
    }
    
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        ki_line = atof(&string);
        i = 0;
    }   
}

void derivative_Linear_menu(void)
{
    extern float kd_line;
    static uint8_t i = 0;
    static int8_t string[10];
    if((keyb > 96 && keyb < 123) || (keyb > 64 && keyb < 91))
    {
        clear_IHM();
        uart.enviar_string("----Menu 6----\nkd_lin: ");
        return; // evitar letras de comandos
    }
        
    if(keyb != '*'){
        string[i] = keyb;
        i++;
    }
    
    else
    {   
        uart.enviar_string(&string); 
        uart.enviar_caractere('\n');
        kd_line = atof(&string);
        i = 0;
    }
}

void menu_7(void)
{
    clear_IHM();
    uart.enviar_string("----Menu 7----\nParar robô? S/N\n");
    if(keyb == 'S')
    {
        start_running = 0;
        motores.frear();
    }
}

void menu_8(void)
{
    clear_IHM();
    uart.enviar_string("----Menu 8----\nIniciar Robô? S/N\n");
    if(keyb == 'S'){
        start_running = 1;
    }
    else if (keyb == 'N'){
        start_running = 0;
    }
}