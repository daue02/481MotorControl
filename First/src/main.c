#include "main.h"
#include "controls.h"
#include "drill_hal.h"
#include "hmi_hal.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"

#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;
UART_HandleTypeDef huart6;

struct stateMachine state = {0};

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *y, double *z);
void SerialDemo(void);
void SystemHealthCheck(void);

uint8_t txData[] = "Hello, Pi!";
uint8_t rxData[32] = {0}; // Buffer to store received data
HAL_StatusTypeDef txStatus, rxStatus;

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == USART1) // Check if the interrupt is from USART1
//   {
//     // Print the received data to the serial monitor
//     printf("Received from Pi: %s\r\n", rxData);

//     // Restart the receive process
//     HAL_UART_Receive_IT(&huart6, rxData, sizeof(txData));
//   }
// }

int main(void)
{
  HAL_Init();
  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  Serial_Init();
  // Motors_Init();
  // Drill_Init();
  // Limit_Switch_Init();
  // HMI_Init();

  while (1)
  {
    // Transmit data to the loopback
    txStatus = HAL_UART_Transmit(&huart6, txData, sizeof(txData), HAL_MAX_DELAY);
    if (txStatus != HAL_OK)
    {
      printf("Transmission failed with status: %d\r\n", txStatus);
    }
    else
    {
      // Wait to receive the data sent
      rxStatus = HAL_UART_Receive(&huart6, rxData, sizeof(txData), HAL_MAX_DELAY);
      if (rxStatus == HAL_OK)
      {
        // Print received data to serial monitor
        printf("Received from Pi: %s\r\n", rxData);
      }
      else
      {
        printf("Receive failed with status: %d\r\n", rxStatus);
      }
    }

    HAL_Delay(1000); // Delay for demonstration purposes
  }
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

//

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
  UartHandle.Instance = USARTx;
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

  // Initialize secondary UART
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart6) != HAL_OK)
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