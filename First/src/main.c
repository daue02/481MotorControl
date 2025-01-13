#include "main.h"
// #include "controls.h"
// #include "drill_hal.h"
// #include "hmi_hal.h"
// #include "motor_hal.h"
#include "uart.h"
// #include "limit_switch_hal.h"

#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;

CommandData currentCommand;

static void SystemClockConfig(void);
void UART_Test(CommandData *cmdData);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *y, double *z);
void SerialDemo(void);

// Encoder
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

// TIM1 Initialization (First Encoder)
void MX_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE(); // Enable TIM1 clock

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0; // No prescaler for full resolution
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF; // Max value for 16-bit counter
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef sConfig = {0};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Both channels for quadrature encoding

  // Channel 1 Configuration (PA8 -> TIM1_CH1)
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0F; // Add a filter to suppress noise

  // Channel 2 Configuration (PA9 -> TIM1_CH2)
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0F; // Add a filter to suppress noise

  // Initialize TIM1 in encoder mode
  HAL_TIM_Encoder_Init(&htim1, &sConfig);

  // Start the TIM1 base
  HAL_TIM_Base_Start(&htim1);
}

// TIM8 Initialization (Second Encoder)
void MX_TIM8_Init(void)
{
  __HAL_RCC_TIM8_CLK_ENABLE(); // Enable TIM8 clock

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0; // No prescaler for full resolution
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xFFFF; // Max value for 16-bit counter
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef sConfig = {0};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Both channels for quadrature encoding

  // Channel 1 Configuration (PC8 -> TIM8_CH3)
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0F; // Add a filter to suppress noise

  // Channel 2 Configuration (PC9 -> TIM8_CH4)
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0F; // Add a filter to suppress noise

  // Initialize TIM8 in encoder mode
  HAL_TIM_Encoder_Init(&htim8, &sConfig);

  // Start the TIM8 base
  HAL_TIM_Base_Start(&htim8);
}

void Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOA clock for TIM1
  // __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure PA8 (TIM1_CH1) and PA9 (TIM1_CH2) for first encoder
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function, push-pull
  GPIO_InitStruct.Pull = GPIO_PULLUP;     // Use pull-up for stability
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // Alternate function for TIM1
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable GPIOC clock for TIM8
  // __HAL_RCC_GPIOC_CLK_ENABLE();

  // Configure PC8 (TIM8_CH3) and PC9 (TIM8_CH4) for second encoder
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function, push-pull
  GPIO_InitStruct.Pull = GPIO_PULLUP;     // Use pull-up for stability
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8; // Alternate function for TIM8
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Initialize TIM1 and TIM8
  MX_TIM1_Init();
  MX_TIM8_Init();
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
  Encoder_Init();

  printf("System Initialized\r\n");

  CommandData cmdData;

  // Blocking connection test
  // UART_Test(&cmdData);

  while (1)
  {
    // Read Encoder 1 position (TIM1)
    int16_t encoder1_position = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t encoder2_position = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    printf("Encoder 1, 2: %ld, %ld\r\n", encoder1_position, encoder2_position);

    HAL_Delay(100); // Delay for readability
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

void UART_Test(CommandData *cmdData)
{
  // uart test
  while (1)
  {
    if (rxReady)
    {
      int status = receiveMessage(cmdData);
      if (status == 0)
      {
        printf("Received Command: Axis %d, Position %d\r\n", cmdData->axis, cmdData->position);
        motorOperationCompleteCallback(currentCommand.axis, currentCommand.position);
      }
    }
    HAL_Delay(1);
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

// /**
//  * @brief Runs a demo which allows the user to send the robot y and z position commands and move the motors.
//  *
//  */
// void SerialDemo(void)
// {
//   printf("---------- Entered Serial Demo ----------\n");
//   while (1)
//   {
//     double y = 0, z = 0;
//     RecieveCoordinates(&y, &z);
//     MoveTo(y, z);
//     while (motorsMoving())
//     {
//       HAL_Delay(1); // Prevent user from sending another request while still moving
//     }
//   }
// }