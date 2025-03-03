#include "uart.h"
#include "encoder_hal.h"
#include "utilities.h"
#include "motor_hal.h"
#include "drill_hal.h"
#include "controls.h"

#define RX_BUFFER_SIZE 4
#define TICK_BUF_SIZE 6
#define BAT_BUF_SIZE 4

uint8_t COMMAND_BYTE = 0x87;
uint8_t ACK_BYTE = 0x43;
uint8_t CALLABACK_BYTE = 0x03;
uint8_t ERROR_BYTE = 0x04;
uint8_t TICKS_BYTE = 0xAE;
uint8_t BATTERY_BYTE = 0x11;
uint8_t LEFT_BYTE = 0x7E;
uint8_t RIGHT_BYTE = 0x3F;
uint8_t DOWN_BYTE = 0x9A;
uint8_t UP_BYTE = 0x5A;
uint8_t DRILL_BYTE = 0x0F;
uint8_t STOP_BYTE = 0x07;

UART_HandleTypeDef huart5;

void UART5_IRQHandler(void);
void constructMessage(uint8_t messageType, uint8_t axis, uint16_t position, uint8_t *txData);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

volatile bool rxReady = false;
volatile bool commandPending = false;
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t ticksBuffer[TICK_BUF_SIZE];
uint8_t batBuffer[BAT_BUF_SIZE];

/**
 * @brief UART5 interrupt handler.
 *
 */
void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}

/**
 * @brief Initializes the UART peripheral and starts reception in interrupt mode.
 *
 * This function configures the UART interface for communication and begins the initial
 * reception process using interrupts. It should be called during system initialization.
 */
void UART_Init()
{
    // Initialize secondary UART (UART5 for Pi connection)
    huart5.Instance = UART5;       // Use UART5
    huart5.Init.BaudRate = 115200; // Set to match Pi's UART baud rate
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    // Enable UART5 interrupt in NVIC
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0); // Priority set to 0 (high) and subpriority 1 (adjust if needed)
    HAL_NVIC_EnableIRQ(UART5_IRQn);         // Enable UART5 interrupt

    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        LOG_ERROR("UART init failed");
        ErrorHandler();
    }

    HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
}

/**
 * @brief Receives and processes a message from the UART interface.
 *
 * This function checks if a message has been received, parses it, and handles it accordingly.
 * If a command message is received, it stores the command data for further processing.
 *
 * @param cmdData Pointer to a CommandData structure where the received command data will be stored.
 * @return 0 if a valid command was received and processed; -1 otherwise.
 */
bool receiveCommand(uint16_t *weedPos)
{
    rxReady = false; // Reset flag

    uint8_t receivedChecksum = rxBuffer[RX_BUFFER_SIZE - 1];
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < RX_BUFFER_SIZE - 1; i++)
    {
        calculatedChecksum += rxBuffer[i];
    }
    calculatedChecksum = calculatedChecksum % 256;

    if (receivedChecksum != calculatedChecksum)
    {
        return false;
    }

    uint16_t position = ((uint16_t)rxBuffer[1] << 8) | rxBuffer[2];
    *weedPos = position;

    if (HAL_UART_Transmit_IT(&huart5, &ACK_BYTE, 1) != HAL_OK)
    {
        LOG_ERROR("Failed to send ACK.");
        return false;
    }

    return true;
}

/**
 * @brief Sends the encoder tick values over the UART interface.
 *
 * This function prepares a buffer containing the encoder tick values for two axes,
 * calculates a checksum for data integrity, and transmits the buffer using the UART interface.
 *
 * @param ticks1 The tick value for the first encoder.
 * @param ticks2 The tick value for the second encoder.
 */
void sendTicks(int16_t ticks1, int16_t ticks2)
{
    // Prepare the buffer
    ticksBuffer[0] = 0xAE; // Start byte

    // Serialize ticks1
    ticksBuffer[1] = (ticks1 >> 8) & 0xFF;
    ticksBuffer[2] = ticks1 & 0xFF;

    // Serialize ticks2
    ticksBuffer[3] = (ticks2 >> 8) & 0xFF;
    ticksBuffer[4] = ticks2 & 0xFF;

    // Checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 5; i++)
    {
        checksum += ticksBuffer[i];
    }
    ticksBuffer[5] = checksum;

    HAL_UART_Transmit_IT(&huart5, ticksBuffer, TICK_BUF_SIZE);
    LOG_DEBUG("Sent encoder ticks: %d, %d", ticks1, ticks2);
}

/**
 * @brief Constructs a message with the specified parameters and calculates the checksum.
 *
 * This function formats a message according to the communication protocol by setting the
 * message type, axis, and position. It also calculates and appends the checksum to ensure
 * data integrity during transmission.
 *
 * @param messageType The type of the message (e.g., command, data, acknowledgment, error).
 * @param axis The axis identifier (e.g., 0, 1, 2).
 * @param position The position value associated with the message.
 * @param buffer Pointer to the buffer where the constructed message will be stored.
 */
void constructMessage(uint8_t messageType, uint8_t axis, uint16_t position, uint8_t *txData)
{
    txData[0] = messageType;
    txData[1] = axis;
    txData[2] = (position >> 8) & 0xFF;                                // Position high byte
    txData[3] = position & 0xFF;                                       // Position low byte
    txData[4] = (txData[0] + txData[1] + txData[2] + txData[3]) % 256; // Checksum
}

/**
 * @brief Callback function invoked when the motor operation is complete.
 *
 * This function is called by the motor module (or simulated via a timer interrupt) when the motor operation
 * has finished. It sends a completion message back to the Raspberry Pi and updates the system state.
 */
void motorOperationCompleteCallback(void)
{
    stopTicksTimer();
    if (HAL_UART_Transmit_IT(&huart5, &CALLABACK_BYTE, 1) != HAL_OK)
    {
        LOG_ERROR("Failed to send callback to Pi.");
    }
    startTicksTimer();

    commandPending = false;

    LOG_INFO("Removal operation complete");
}

/**
 * @brief UART transmit complete callback function.
 *
 * This function is called by the HAL library when a UART transmit operation completes.
 * It can be used to perform post-transmission tasks if needed.
 *
 * @param huart Pointer to the UART handle that triggered the callback.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // Transmission complete; you can set a flag here if needed
    }
}

/**
 * @brief UART receive complete callback function.
 *
 * This function is called by the HAL library when a UART receive operation completes.
 * It sets a flag to indicate that data is ready to be processed by the main loop.
 *
 * @param huart Pointer to the UART handle that triggered the callback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        static uint8_t tempBuffer[RX_BUFFER_SIZE];
        static uint8_t bytesReceived = 0;

        tempBuffer[bytesReceived++] = rxBuffer[0];

        // Single-byte request for encoder data
        if (tempBuffer[0] == TICKS_BYTE && bytesReceived == 1)
        {
            int16_t ticks1, ticks2;
            getTicks(&ticks1, &ticks2);
            sendTicks(ticks1, ticks2);

            // LOG_INFO("Sent encoder ticks: %ld, %ld", ticks1, ticks2);

            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Battery voltage request
        else if (tempBuffer[0] == BATTERY_BYTE && bytesReceived == 1)
        {
            float voltage = readBatteryVoltage(&bat);
            batBuffer[0] = 0x11;
            batBuffer[1] = voltage;
            batBuffer[2] = (int)(voltage * 100) % 100;
            batBuffer[3] = (batBuffer[0] + batBuffer[1] + batBuffer[2]) % 256;

            HAL_UART_Transmit_IT(&huart5, batBuffer, BAT_BUF_SIZE);

            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Move y-axis negative (left)
        else if (tempBuffer[0] == LEFT_BYTE && bytesReceived == 1)
        {
            LOG_INFO("LEFT");

            MoveBySpeed(&motorY, -motorY.speed / motorY.lead * 60);
            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Move y-axis positive (right)
        else if (tempBuffer[0] == RIGHT_BYTE && bytesReceived == 1)
        {
            LOG_INFO("RIGHT");

            MoveBySpeed(&motorY, motorY.speed / motorY.lead * 60);
            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Move z-axis negative (down)
        else if (tempBuffer[0] == DOWN_BYTE && bytesReceived == 1)
        {
            LOG_INFO("DOWN");

            MoveBySpeed(&motorZ, motorZ.speed / motorZ.lead * 60);
            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Move z-axis positive (up)
        else if (tempBuffer[0] == UP_BYTE && bytesReceived == 1)
        {
            LOG_INFO("UP");

            MoveBySpeed(&motorZ, -motorZ.speed / motorZ.lead * 60);
            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Start drill
        else if (tempBuffer[0] == DRILL_BYTE && bytesReceived == 1)
        {
            LOG_INFO("DRILL");

            setDrillPower(50);
            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }
        // Stop
        else if (tempBuffer[0] == STOP_BYTE && bytesReceived == 1)
        {
            LOG_INFO("STOP");

            StopMotors();
            setDrillPower(0);
            updateStateMachine("Unhomed");

            bytesReceived = 0;
            HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
            return;
        }

        // Multi-byte command
        if (bytesReceived == RX_BUFFER_SIZE)
        {
            // Copy full message into rxBuffer for processing
            memcpy(rxBuffer, tempBuffer, RX_BUFFER_SIZE);

            bytesReceived = 0;
            rxReady = true;
        }

        HAL_UART_Receive_IT(&huart5, rxBuffer, 1);
    }
}
