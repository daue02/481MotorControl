#ifndef ENCODER_HAL_H
#define ENCODER_HAL_H

#include "main.h"

void Encoder_Init(void);
void getTicks(int16_t *ticks1, int16_t *ticks2);
void stopTicksTimer(void);
void startTicksTimer(void);
#endif