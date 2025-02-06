// Define to prevent recursive inclusion
#ifndef __BATTERY_HEALTH_H
#define __BATTERY_HEALTH_H

#include "main.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *adcPort;
    uint16_t adcPin;
    uint16_t adcChannel;
    float R1;
    float R2;
    float V_REF;
} Battery;

extern Battery bat;

void Battery_Health_Init(void);
float readBatteryVoltage(Battery *bat);

#endif