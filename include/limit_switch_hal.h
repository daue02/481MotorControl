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
} LimitSwitch;

extern LimitSwitch ySW_neg;
extern LimitSwitch ySW_pos;
extern LimitSwitch zSW_neg;
extern LimitSwitch zSW_pos;
extern LimitSwitch piSW;

void Limit_Switch_Init(void);

#endif