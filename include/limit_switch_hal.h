#ifndef __LIMIT_SWITCH_H
#define __LIMIT_SWITCH_H

#include "main.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState Pin_state;
    uint32_t lastDebounceTime;
} InterruptSwitch;

extern InterruptSwitch ySW_neg;
extern InterruptSwitch ySW_pos;
extern InterruptSwitch zSW_neg;
extern InterruptSwitch zSW_pos;
extern InterruptSwitch piSW;

void Limit_Switch_Init(void);

#endif