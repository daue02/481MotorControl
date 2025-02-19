// Define to prevent recursive inclusion
#ifndef __DRILL_HAL_H
#define __DRILL_HAL_H

#include "main.h"
#include "stm32f4xx_hal_tim.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *pwmPort;
    uint16_t pwmPin;
    int targetPower;
    int currentPower;
    double accel; // % per sec
} Drill;

extern Drill motorDrill;

void Drill_Init(void);
void setDrillPower(int power);
bool isDrillPWMDisabled(void);

#endif