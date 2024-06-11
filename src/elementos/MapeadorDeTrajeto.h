#ifndef MAPEAMENTO_H
#define MAPEAMENTO_H

#include <stdint.h>

struct MapeadorDeTrajeto {
    void (*iniciar)(void);
    void (*finalizar)(void);
    void (*atualizar)(void);
} mapeador;

#endif 