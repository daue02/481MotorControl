#include "main.h"
#include "utilities.h"
#include "controls.h"
#include "drill_hal.h"
#include "encoder_hal.h"
#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "uart.h"

#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;

struct stateMachine state = {0};

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *y, double *z);
void SerialDemo(void);

// #define TEST_1 // Radial Accuracy Test
// #define TEST_2 // Penetration Depth Test
// #define HOMO_CAL // Homography calibration routine

int main(void)
{

  HAL_Init();
  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // Wait for Pi to boot to avoid unintentional movement
  Serial_Init();
  printf("\n\n\n\n\n"); // Readability between resets
  Motors_Init();
  HAL_GPIO_WritePin(motorY.sleepPort, motorY.sleepPin, GPIO_PIN_RESET); // Unstall y motor
  HAL_GPIO_WritePin(motorZ.sleepPort, motorZ.sleepPin, GPIO_PIN_RESET); // Unstall z motor
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET)
  {
    HAL_Delay(10);
  }

  Drill_Init();
  Encoder_Init();
  Limit_Switch_Init();
  UART_Init();
  Utilities_Init();

  LOG_INFO("System Initialized");
  updateStateMachine("Unhomed");
  SystemHealthCheck();

  while (1)
  {
    if (rxReady)
    {
      uint16_t weedPos;
      if (receiveCommand(&weedPos))
      {
        if (!commandPending)
        {
          commandPending = true;

// Radial Accuracy Test
#ifdef TEST_1
          LOG_INFO("Radial Accuracy Test Activated");
          SystemHealthCheck();
          if (state.unhomed)
          {
            HomeMotors();
          }
          updateStateMachine("Positioning");
          MoveTo(weedPos, 10);
          LOG_INFO("Movement complete - Please measure accuracy");
          updateStateMachine("Waiting");
          // Not calling the motor callback prevents Pi from taking over
#endif

// Penetration Depth Test
#ifdef TEST_2
          LOG_INFO("Penetration Depth Test Activated");
          SystemHealthCheck();
          if (state.unhomed)
          {
            HomeMotors();
          }
          locateWeed(weedPos);
          updateStateMachine("Drilling");
          setDrillPower(50, DRILLCW); // Tweak drill power as required
          MoveTo(weedPos, motorZ.posMin);
          setDrillPower(0, DRILLCW);
          updateStateMachine("Waiting");
          LOG_INFO("Mark location on drillbit for measurement - In the next 15 seconds");
          HAL_Delay(15000);
          updateStateMachine("Positioning");
          MoveTo(weedPos, 25);
          LOG_INFO("Movement complete - Please measure hole depth on drill bit");
          // Not calling the motor callback prevents Pi from taking over
#endif

// Homography calibration
#ifdef HOMO_CAL
          LOG_INFO("Homography Calibration Routine Activated");
          SystemHealthCheck();
          if (state.unhomed)
          {
            HomeMotors();
          }
          updateStateMachine("Positioning");
          MoveTo(weedPos, 0);
          updateStateMachine("Waiting");
          LOG_INFO("Reset Nucleo when complete");
          while (1)
          {
            HAL_Delay(1);
          }
#endif

          LOG_INFO("Automatic sequence activated");
          SystemHealthCheck();
          if (state.unhomed)
          {
            HomeMotors();
          }
          locateWeed(weedPos);
          removeWeed(weedPos, "fake");
        }
        else
        {
          // Handle case where a command is received while another is pending
          LOG_WARN("Command received while another is in progress. Ignoring or queueing.");
        }
      }
    }
    HAL_Delay(1);
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
    LOG_ERROR("ClockConfig - Enabling HSI Oscillator - Failed");
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
    LOG_ERROR("ClockConfig - Configuring HCLK, PCKL1, PCKL2 - Failed");
    ErrorHandler();
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
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    LOG_ERROR("Serial Initialization Failed");
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
  LOG_INFO("Enter desired Y coordinate [mm]: ");
  *y = ReceiveFloat();
  LOG_INFO("");
  LOG_INFO("Enter desired Z coordinate [mm]: ");
  *z = ReceiveFloat();
  LOG_INFO("");
}

/**
 * @brief Runs a demo which allows the user to send the robot y and z position commands and move the motors.
 *
 */
void SerialDemo(void)
{
  LOG_INFO("---------- Entered Serial Demo ----------");
  while (1)
  {
    double y = 0, z = 0;
    RecieveCoordinates(&y, &z);
    MoveTo(y, z);
    while (motorsMoving())
    {
      HAL_Delay(1); // Prevent user from sending another request while still moving
    }
  }
}
