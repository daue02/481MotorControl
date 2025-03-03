# This script fixes the issue where the Nucleo MCU was 
# getting jammed up after multiple uploads. It automatically 
# erases the Nucleo before flashing each time

Import("env") # type: ignore
env.Replace(  # type: ignore
    UPLOADER="openocd",
    UPLOADERFLAGS=[
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", "init",
        "-c", "reset halt",
        "-c", "stm32f4x mass_erase 0",
        "-c", "shutdown"
    ]
)
