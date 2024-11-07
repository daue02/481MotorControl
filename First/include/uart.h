#ifndef __UART_H
#define __UART_H

#include "main.h"

typedef struct
{
    uint8_t axis;
    uint16_t position;
} CommandData;

extern volatile bool rxReady;
extern volatile bool commandPending;

void UART_Init();
int receiveMessage(CommandData *cmdData);
void motorOperationCompleteCallback(uint8_t axis, uint16_t position);

#endif
