// Define to prevent recursive inclusion
#ifndef __DRILL_HAL_H
#define __DRILL_HAL_H

#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "utilities.h"

#define DRILLCW 1
#define DRILLCCW 0

typedef struct
{
    const char *name;
    GPIO_TypeDef *pwmPort;
    uint16_t pwmPin;
    GPIO_TypeDef *dirPort;
    uint16_t dirPin;
    int targetPower;
    int currentPower;
    int dir;
    double accel; // % per sec
} Drill;

extern Drill motorDrill;

void Drill_Init(void);
void setDrillPower(int power, int dir);
bool isDrillPWMDisabled(void);

#endif