// Define to prevent recursive inclusion
#ifndef __UTILITIES_H
#define __UTILITIES_H

#include "main.h"
#include <string.h>

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

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pin_state;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
    bool latched;
} LED;

extern Battery bat;
extern LED greenLED;
extern LED redLED;
extern LED activeLED;

void Utilities_Init(void);
float readBatteryVoltage(Battery *bat);
void SystemHealthCheck(void);
void ErrorHandler(void);
void changeLEDState(LED butLED, const char *ledMode);

#endif