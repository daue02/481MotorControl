#ifndef __UART_H
#define __UART_H

#include "main.h"

extern volatile bool rxReady;
extern volatile bool commandPending;

void UART_Init();
bool receiveCommand(uint16_t *weedPos);
void motorOperationCompleteCallback(void);
void sendTicks(int16_t ticks1, int16_t ticks2);

/*
    Notes about using this module:
        - It is assumed that the Pi will send a command and then
            wait for an acknowledgment, and a replay command before
            sending another command.
        - The Nucleo will not send commands without receiving a command
            first.
        - motorOperationCompleteCallback should be called by the motor
            HAL when the motor operation is complete which will send a
            completion message back to the Pi.
 */

#endif
