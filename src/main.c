#include "main.h"
#include "battery_health.h"
#include "controls.h"
#include "drill_hal.h"
#include "encoder_hal.h"
#include "hmi_hal.h"
#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "uart.h"

#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;

struct stateMachine state = {0};
CommandData currentCommand;

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *y, double *z);
void SerialDemo(void);
void SystemHealthCheck(void);

int main(void)
{
  HAL_Init();
  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  Serial_Init();
  Battery_Health_Init();
  Drill_Init();
  Encoder_Init();
  HMI_Init();
  Limit_Switch_Init();
  Motors_Init();
  UART_Init();

  LOG_INFO("System Initialized");
  float voltage = readBatteryVoltage(&bat);
  LOG_INFO("Battery Voltage: %d.%02d", (int)voltage, (int)(voltage * 100) % 100);

  // CommandData cmdData;
  currentCommand.position = 125; // REMOVE

  SystemHealthCheck();
  HomeMotors();

  while (1)
  {
    if (1) // Automatic sequence
    {
      LOG_INFO("Automatic sequence activated");
      SystemHealthCheck();
      updateStateMachine("Positioning");
      MoveTo(currentCommand.position, -25);
      updateStateMachine("Drilling");
      // Start the drill motion here
      MoveTo(currentCommand.position, motorZ.posMax);
      // Stop the drill motion here
      updateStateMachine("Positioning");
      HAL_Delay(500);
      MoveTo(currentCommand.position, -25);
      MoveTo(motorY.posMin, motorZ.posMin);
      updateStateMachine("Waiting");
      HAL_Delay(5000);
    }

    /*
    if (rxReady)
    {
      int status = receiveMessage(&cmdData);
      if (status == 0)
      {
        if (!commandPending)
        {
          currentCommand = cmdData;
          commandPending = true;

          // Check battery voltage each time we recieve a position command
          // If too low, send robot to faulted state
          float voltage = readBatteryVoltage(&bat);
          if (voltage < bat.V_MIN)
          {
            LOG_ERROR("Battery Voltage LOW: %d.%02d", (int)voltage, (int)(voltage * 100) % 100);
            ErrorHandler();
          }

          if (1) // Automatic sequence
          {
            LOG_INFO("Automatic sequence activated");
            SystemHealthCheck();
            HomeMotors();
            updateStateMachine("Positioning");
            MoveTo(currentCommand.position, -25);
            updateStateMachine("Drilling");
            // Start the drill motion here
            MoveTo(currentCommand.position, 100);
            // Stop the drill motion here
            updateStateMachine("Positioning");
            HAL_Delay(500);
            MoveTo(currentCommand.position, -25);
            MoveTo(50, -190);
            // Would add the bit-clearing stuff here
            updateStateMachine("Waiting");
          }
          else // Manual sequence
          {
            LOG_INFO("Manual sequence activated");
            // Manual mode to be added
          }
        }
        else
        {
          // Handle case where a command is received while another is pending
          LOG_WARN("Command received while another is in progress. Ignoring or queueing.");
        }
      }
    }

    // Wait until motor movement is complete before starting the next command
    if (commandPending)
    {
      if (motorsMoving())
      {
        HAL_Delay(1);
      }
      motorOperationCompleteCallback(currentCommand.axis, currentCommand.position);
    }

    HAL_Delay(1);
  */
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

/**
 * @brief Put all system health checks here
 *
 */
void SystemHealthCheck(void)
{
  // Check that all limit switches are closed (NC switched).
  if (ySW_pos.Pin_state)
  {
    LOG_ERROR("Error: check Y+ sw");
  }
  else if (ySW_neg.Pin_state)
  {
    LOG_ERROR("Error: check Y- sw");
  }
  else if (zSW_pos.Pin_state)
  {
    LOG_ERROR("Error: check Z+ sw");
  }
  else if (zSW_neg.Pin_state)
  {
    LOG_ERROR("Error: check Z- sw");
  }
  else
  {
    updateStateMachine("Unhomed");
    return;
  }
  ErrorHandler();
}