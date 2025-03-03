# This script fixes the issue where the Nucleo MCU was 
# getting jammed up after multiple uploads. It automatically 
# erases the Nucleo before flashing each time

Import("env")
import subprocess
import time

# Unlock MCU, erase flash, and reset
subprocess.run([
    "openocd",
    "-f", "interface/stlink.cfg",
    "-f", "target/stm32f4x.cfg",
    "-c", "init",
    "-c", "reset halt",
    "-c", "stm32f4x unlock 0",  # Force unlock on every upload
    "-c", "stm32f4x mass_erase 0",
    "-c", "shutdown"
])

# Short delay to ensure MCU is fully reset
time.sleep(2)

# Perform a hardware reset using ST-Link
subprocess.run(["st-flash", "reset"])


