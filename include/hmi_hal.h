#ifndef __HMI_H
#define __HMI_H

#include "main.h"
#include <string.h>

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
} buttonLED;

extern buttonLED greenLED;
extern buttonLED redLED;
extern buttonLED activeLED;
extern buttonLED homeButton;
extern buttonLED auxButton;

void HMI_Init(void);
void changeLEDState(buttonLED butLED, const char *ledMode);
void hmiTesting(void);

#endif /* __HMI_H */