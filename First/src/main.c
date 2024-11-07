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

void UART_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
bool parse_message(uint8_t *rxData, uint8_t *message_type, uint8_t *axis, uint16_t *position);
void construct_message(uint8_t message_type, uint8_t axis, uint16_t position, uint8_t *txData);

uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[TX_BUFFER_SIZE];
bool rxReady = false;

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

  while (1)
  {
    if (rxReady)
    {
      rxReady = false;

      uint8_t message_type;
      uint8_t axis;
      uint16_t position;
      bool valid = parse_message(rxBuffer, &message_type, &axis, &position);

      if (valid)
      {
        if (message_type == 0x01) // Command message
        {
          // Process the received command
          printf("Received Command: Axis %d, Position %d\r\n", axis, position);

          // Send acknowledgment back
          construct_message(0x03, axis, position, txBuffer); // Message Type 0x03 for ACK
          HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
        }
        else if (message_type == 0x03 || message_type == 0x04)
        {
          // Received ACK or Error; typically, the Nucleo would not receive these in response to commands it didn't send
          printf("Received Message Type %d, ignoring.\r\n", message_type);
          // Do not send acknowledgment to prevent ack loops
        }
        else if (message_type == 0x02) // Data message
        {
          // Process data message as needed
          printf("Received Data: Axis %d, Position %d\r\n", axis, position);

          // Send acknowledgment back
          construct_message(0x03, axis, position, txBuffer); // ACK
          HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
        }
        else
        {
          printf("Unknown message type received: %d\r\n", message_type);
          // Optionally send an error message back
          construct_message(0x04, 0, 0, txBuffer); // Error message
          HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
        }
      }
      else
      {
        printf("Checksum error\r\n");
        // Send error message back
        construct_message(0x04, 0, 0, txBuffer); // Error message
        HAL_UART_Transmit_IT(&huart5, txBuffer, TX_BUFFER_SIZE);
      }

      // Restart UART reception
      HAL_UART_Receive_IT(&huart5, rxBuffer, RX_BUFFER_SIZE);
    }

    // other work
    HAL_Delay(1);
  }
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

void construct_message(uint8_t message_type, uint8_t axis, uint16_t position, uint8_t *txData)
{
  txData[0] = message_type;
  txData[1] = axis;
  txData[2] = (position >> 8) & 0xFF;                                // Position high byte
  txData[3] = position & 0xFF;                                       // Position low byte
  txData[4] = (txData[0] + txData[1] + txData[2] + txData[3]) % 256; // Checksum
}

// Function to parse a received message
bool parse_message(uint8_t *rxData, uint8_t *message_type, uint8_t *axis, uint16_t *position)
{
  uint8_t received_checksum = rxData[4];
  uint8_t calculated_checksum = (rxData[0] + rxData[1] + rxData[2] + rxData[3]) % 256;

  if (received_checksum != calculated_checksum)
  {
    // Checksum error
    return false;
  }

  *message_type = rxData[0];
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