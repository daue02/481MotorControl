#ifndef __LIMIT_SWITCH_H
#define __LIMIT_SWITCH_H

#include "main.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t Pin_p;
    GPIO_PinState Pin_p_state;
    uint16_t Pin_n;
    GPIO_PinState Pin_n_state;
} LimitSwitch;

extern LimitSwitch theta1SW;
extern LimitSwitch theta2SW;
extern LimitSwitch thetazSW;

void Limit_Switch_Init(void);

#endif /* __LIMIT_SWITCH_H */