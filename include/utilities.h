// Define to prevent recursive inclusion
#ifndef __UTILITIES_H
#define __UTILITIES_H

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
    float V_MIN;
} Battery;

extern Battery bat;

void Battery_Health_Init(void);
float readBatteryVoltage(Battery *bat);
void SystemHealthCheck(void);
void ErrorHandler(void);

#endif