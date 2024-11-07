#include "main.h"
#include "controls.h"
#include "drill_hal.h"
#include "hmi_hal.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"

#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;
UART_HandleTypeDef huart5;

struct stateMachine state = {0};

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *y, double *z);
void SerialDemo(void);
void SystemHealthCheck(void);

#include <stdbool.h>
#include <stdint.h>

#define RX_BUFFER_SIZE 5
#define TX_BUFFER_SIZE 5

typedef struct
{
  uint8_t axis;
  uint16_t position;
} CommandData;

void UART_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
bool parseMessage(uint8_t *rxData, uint8_t *messageType, uint8_t *axis, uint16_t *position);
void constructMessage(uint8_t messageType, uint8_t axis, uint16_t position, uint8_t *txData);
int receiveMessage(CommandData *cmdData);

void motorOperationCompleteCallback(uint8_t axis, uint16_t position);

uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[TX_BUFFER_SIZE];
bool rxReady = false;

CommandData currentCommand;
volatile bool commandPending = false;

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

int main(void)
{
  HAL_Init();
  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  Serial_Init();
  UART_Init();
  Drill_Init();
  Limit_Switch_Init();
  HMI_Init();

  printf("System Initialized\r\n");

  CommandData cmdData;

  while (1)
  {
    if (rxReady)
    {
      int status = receiveMessage(&cmdData);
      if (status == 0)
      {
        if (!commandPending)
        {
          // Start motor operation asynchronously
          currentCommand = cmdData;
          commandPending = true;

          // Call motor operation function here
        }
        else
        {
          // Handle case where a command is received while another is pending
          printf("Command received while another is in progress. Ignoring or queueing.\r\n");
        }
      }
    }

    /*
      This is to simulate the motor stuf completing elsewhere
      and then calling the callback function to indicate completion.
      This should actaully be done in the motor HAL once the movement is complete.
    */
    if (commandPending)
    {
      HAL_Delay(1000); // Simulate motor operation
      motorOperationCompleteCallback(currentCommand.axis, currentCommand.position);
    }

    // Other work can be done here
    HAL_Delay(1);
  }
}

void sendMessage(uint8_t messageType, uint8_t axis, uint16_t position)
{
  constructMessage(messageType, axis, position, txBuffer);
  HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
  printf("Sent Message: Type %d, Axis %d, Position %d\r\n", messageType, axis, position);
}

int receiveMessage(CommandData *cmdData)
{
  rxReady = false;

  uint8_t messageType;
  uint8_t axis;
  uint16_t position;
  int returnCode = -1; // Default to -1, change to 0 if a valid command is received

  bool valid = parseMessage(rxBuffer, &messageType, &axis, &position);

  if (valid)
  {
    if (messageType == 0x01) // Command message from Raspberry Pi
    {
      // Process the received command
      printf("Received Command: Axis %d, Position %d\r\n", axis, position);

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
      printf("Received Message Type %d, ignoring.\r\n", messageType);
    }
    else if (messageType == 0x02) // Data message (unlikely in this context)
    {
      // Process data message as needed
      printf("Received Data: Axis %d, Position %d\r\n", axis, position);

      // Send acknowledgment back
      constructMessage(0x03, axis, position, txBuffer); // ACK
      HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
    }
    else
    {
      printf("Unknown message type received: %d\r\n", messageType);
      // Optionally send an error message back
      constructMessage(0x04, 0, 0, txBuffer); // Error message
      HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
    }
  }
  else
  {
    printf("Checksum error\r\n");
    // Send error message back
    constructMessage(0x04, 0, 0, txBuffer); // Error message
    HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
  }

  // Restart UART reception
  HAL_UART_Receive_IT(&huart5, rxBuffer, RX_BUFFER_SIZE);

  return returnCode;
}

void motorOperationCompleteCallback(uint8_t axis, uint16_t position)
{
  // Send a message back to the Pi to indicate completion
  sendMessage(0x02, axis, position); // Message Type 0x02 for Data

  // Update command pending flag
  commandPending = false;

  printf("Motor operation complete: Axis %d, Position %d\r\n", axis, position);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    // Transmission complete; you can set a flag here if needed
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    rxReady = true;
    // Do not restart reception here; handle it after processing
  }
}

void constructMessage(uint8_t messageType, uint8_t axis, uint16_t position, uint8_t *txData)
{
  txData[0] = messageType;
  txData[1] = axis;
  txData[2] = (position >> 8) & 0xFF;                                // Position high byte
  txData[3] = position & 0xFF;                                       // Position low byte
  txData[4] = (txData[0] + txData[1] + txData[2] + txData[3]) % 256; // Checksum
}

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

void UART_Init()
{
  // Initialize secondary UART (UART5 for Pi connection)
  huart5.Instance = UART5;     // Use UART5
  huart5.Init.BaudRate = 9600; // Set to match Pi's UART baud rate
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

  HAL_UART_Receive_IT(&huart5, rxBuffer, sizeof(rxBuffer));
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
 * @brief Configures the clock settings.
 *
 */
void SystemClockConfig(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    ErrorHandler();
  }

  /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if (ret != HAL_OK)
  {
    while (1)
    {
      ;
    }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    ErrorHandler();
  }
}

/**
 * @brief Will hold the device in an infinte loop on error.
 *
 */
void ErrorHandler(void)
{
  updateStateMachine("Faulted");
  while (1)
  {
  }
}

/**
 * @brief Initializes UART for printing to the serial monitor.
 *
 */
void Serial_Init(void)
{
  // Initialize primary UART (e.g., USARTx)
  UartHandle.Instance = USARTx; // Replace USARTx with your primary UART instance
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    ErrorHandler();
  }
}

/**
 * @brief Halts execution until a return character is entered into the serial monitor. Tries to convert the input into a double.
 *
 * @return double - serial monitor input
 */
double ReceiveFloat(void)
{
  char inputBuffer[INPUT_BUFFER_SIZE];
  uint8_t receivedChar;
  double receivedFloat;

  memset(inputBuffer, 0, INPUT_BUFFER_SIZE); // Clear input buffer
  int bufferIndex = 0;

  while (1)
  {
    // Receive a single character
    if (HAL_UART_Receive(&UartHandle, &receivedChar, 1, 0xFFFF) == HAL_OK)
    {
      // Check for end of number input (e.g., newline character)
      if (receivedChar == '\n' || receivedChar == '\r')
      {
        inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
        break;                           // Exit the loop
      }
      else
      {
        // Store the received character into the buffer
        if (bufferIndex < INPUT_BUFFER_SIZE - 1) // Prevent buffer overflow
        {
          inputBuffer[bufferIndex++] = receivedChar;
        }
      }
    }
  }
  // Convert the received string to a floating-point number
  receivedFloat = atof(inputBuffer);

  return receivedFloat;
}

/**
 * @brief Halts program execution and asks user to input an x and a y coordinate.
 *
 * @param y pointer to y coordinate [mm]
 * @param z pointer to z cordinate [mm]
 */
void RecieveCoordinates(double *y, double *z)
{
  printf("Enter desired Y coordinate [mm]: \n");
  *y = ReceiveFloat();
  printf("\n");
  printf("Enter desired Z coordinate [mm]: \n");
  *z = ReceiveFloat();
  printf("\n");
}

/**
 * @brief Runs a demo which allows the user to send the robot y and z position commands and move the motors.
 *
 */
void SerialDemo(void)
{
  printf("---------- Entered Serial Demo ----------\n");
  while (1)
  {
    double y = 0, z = 0;
    RecieveCoordinates(&y, &z);
    MoveTo(y, z, 250.0, 250.0);
    // MoveBy(y,z,500,500);
    while (motorY.isMoving || motorZ.isMoving)
    {
      HAL_Delay(1); // Prevent user from sending another request while still moving
    }
  }
}

/**
 * @brief Put all system health checks here
 *
 */
void SystemHealthCheck(void)
{
  // Check that all limit switches are closed (NC switched).
  if (ySW.Pin_p_state)
  {
    printf("Error: check Y+ sw\n");
  }
  else if (ySW.Pin_n_state)
  {
    printf("Error: check Y- sw\n");
  }
  else if (zSW.Pin_p_state)
  {
    printf("Error: check Z+ sw\n");
  }
  else if (zSW.Pin_n_state)
  {
    printf("Error: check Z- sw\n");
  }
  else
  {
    return;
  }
  ErrorHandler();
}