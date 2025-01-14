
# WeedWarden - Nucleo Firmware

[TODO: add description]

---

## **Features**
- **Motor Control**:
  - Supports homing, positioning, and automatic sequences.
  - Integrated health checks for motors and limit switches.

- **Odometry and Encoder Support**:
  - Uses encoder feedback to compute precise position and motion data.

- **UART Communication**:
  - Communicates with a higher-level controller (e.g., a Raspberry Pi).
  - Supports sending encoder ticks and receiving high-level commands.

- **Automatic and Manual Modes**:
  - Automates specific sequences or allows manual operation via commands.

- **Logging Framework**:
  - Configurable logging with support for debug, info, warning, and error levels.

---

## **Project Structure**

### **Core Files** [TODO: Update]
- `main.c`: Entry point of the firmware. Manages system initialization, command processing, and control logic.
- `log.h`: Logging macros for debugging and performance tuning.
- `encoder_hal.c/h`: Manages encoder input and tick computations.
- `motor_hal.c/h`: Provides motor control functions like homing and movement.
- `uart.c/h`: Handles UART communication with external devices.
- `limit_switch_hal.c/h`: Monitors limit switches for safe operation.
- `drill_hal.c/h`: Manages drill-specific operations.

### **Utilities**
- **Health Checks**:
  - `SystemHealthCheck()` ensures all critical components are functioning correctly before operation.
- **Debugging**:
  - Configurable logging levels allow for detailed debugging during development and minimal output in production.

---

## **How to Build and Deploy**
### **Requirements**
- **Hardware**:
  - STM32 Nucleo Board (F446RE or similar).

- **Software**:
  - [PlatformIO](https://platformio.org/) in Visual Studio Code.
  - Serial terminal for debugging.

### **Building the Project**
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <project-directory>
   ```

2. Open the project in your VsCode

3. Build: `CTRL + ALT + B`

4. Flash: `CTRL + ALT + U`

---

## **Logging**
- Logging is managed via `log.h`.
- **Global Log Levels**:
  - `LOG_LEVEL_NONE`: Disable all logging.
  - `LOG_LEVEL_ERROR`: Log only errors.
  - `LOG_LEVEL_WARN`: Log warnings and errors.
  - `LOG_LEVEL_INFO`: Log informational messages, warnings, and errors.
  - `LOG_LEVEL_DEBUG`: Log all messages, including debug information.

- To change the log level:
  ```c
  #define GLOBAL_LOG_LEVEL LOG_LEVEL_INFO
  ```