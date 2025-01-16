#include "uart.h"
#include "encoder_hal.h"

#define RX_BUFFER_SIZE 5
#define TX_BUFFER_SIZE 5
#define TICK_BUF_SIZE 6

UART_HandleTypeDef huart5;

void UART5_IRQHandler(void);
void sendMessage(uint8_t messageType, uint8_t axis, uint16_t position);
void constructMessage(uint8_t messageType, uint8_t axis, uint16_t position, uint8_t *txData);
bool parseMessage(uint8_t *rxData, uint8_t *messageType, uint8_t *axis, uint16_t *position);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

volatile bool rxReady = false;
volatile bool commandPending = false;
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[TX_BUFFER_SIZE];
uint8_t ticksBuffer[TICK_BUF_SIZE];

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
int receiveMessage(CommandData *cmdData)
{
    rxReady = false;

    uint8_t messageType;
    uint8_t axis;
    uint16_t position;
    int returnCode = -1;

    bool valid = parseMessage(rxBuffer, &messageType, &axis, &position);

    if (valid)
    {
        if (messageType == 0x01) // Command message from Raspberry Pi
        {
            LOG_INFO("Received Command: Axis %d, Position %d", axis, position);

            // Send acknowledgment back
            constructMessage(0x03, axis, position, txBuffer); // Message Type 0x03 for ACK
            HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);

            // Store received command data
            cmdData->axis = axis;
            cmdData->position = position;

            returnCode = 0; // Indicate success
        }
        else if (messageType == 0x03 || messageType == 0x04)
        {
            // Received ACK or Error; ignore or handle as needed
            LOG_INFO("Received Message Type %d, ignoring.", messageType);
        }
        else if (messageType == 0x02) // Data message (unlikely in this context)
        {
            // Process data message as needed
            LOG_INFO("Received Data: Axis %d, Position %d", axis, position);

            // Send acknowledgment back
            constructMessage(0x03, axis, position, txBuffer); // ACK
            HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
        }
        else
        {
            LOG_WARN("Unknown message type received: %d", messageType);
            // Send an error message back
            constructMessage(0x04, 0, 0, txBuffer); // Error message
            HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
        }
    }
    else
    {
        LOG_ERROR("Checksum error");
        // Send error message back
        constructMessage(0x04, 0, 0, txBuffer); // Error message
        HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
    }

    // Restart UART reception
    HAL_UART_Receive_IT(&huart5, rxBuffer, RX_BUFFER_SIZE);

    return returnCode;
}

/**
 * @brief Sends a message over the UART interface.
 *
 * This function constructs a message with the specified parameters and transmits it using
 * the UART interface. It is used to send acknowledgments, data messages, or error messages
 * back to the Raspberry Pi.
 *
 * @param messageType The type of the message to send (e.g., command, data, acknowledgment, error).
 * @param axis The axis identifier associated with the message.
 * @param position The position value associated with the message.
 */
void sendMessage(uint8_t messageType, uint8_t axis, uint16_t position)
{
    constructMessage(messageType, axis, position, txBuffer);
    HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
    LOG_INFO("Sent Message: Type %d, Axis %d, Position %d", messageType, axis, position);
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
 * @brief Parses a received message and verifies its checksum.
 *
 * This function extracts the message type, axis, and position from the received message buffer.
 * It verifies the checksum to ensure that the message has not been corrupted during transmission.
 *
 * @param buffer Pointer to the buffer containing the received message.
 * @param messageType Pointer to a variable where the message type will be stored.
 * @param axis Pointer to a variable where the axis identifier will be stored.
 * @param position Pointer to a variable where the position value will be stored.
 * @return true if the message is valid and the checksum matches; false otherwise.
 */
bool parseMessage(uint8_t *rxData, uint8_t *messageType, uint8_t *axis, uint16_t *position)
{
    uint8_t receivedChecksum = rxData[4];
    uint8_t calculatedChecksum = (rxData[0] + rxData[1] + rxData[2] + rxData[3]) % 256;

    if (receivedChecksum != calculatedChecksum)
    {
        // Checksum error
        return false;
    }

    *messageType = rxData[0];
    *axis = rxData[1];
    *position = ((uint16_t)rxData[2] << 8) | rxData[3];

    return true;
}

/**
 * @brief Callback function invoked when the motor operation is complete.
 *
 * This function is called by the motor module (or simulated via a timer interrupt) when the motor operation
 * has finished. It sends a completion message back to the Raspberry Pi and updates the system state.
 *
 * @param axis The axis identifier for which the motor operation was performed.
 * @param position The position value to which the motor moved.
 */
void motorOperationCompleteCallback(uint8_t axis, uint16_t position)
{
    // Send a message back to the Pi to indicate completion
    sendMessage(0x02, axis, position); // Message Type 0x02 for Data

    commandPending = false;

    LOG_INFO("Motor operation complete: Axis %d, Position %d", axis, position);
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
        if (tempBuffer[0] == 0xAE && bytesReceived == 1)
        {
            int16_t ticks1, ticks2;
            getTicks(&ticks1, &ticks2);
            sendTicks(ticks1, ticks2);

            // LOG_INFO("Sent encoder ticks: %ld, %ld", ticks1, ticks2);

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
