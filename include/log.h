// log.h
#ifndef LOG_H
#define LOG_H

#include <stdio.h>

// Logging levels
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4

// Set global log level
#define GLOBAL_LOG_LEVEL LOG_LEVEL_INFO

// Logging macros
#define LOG_DEBUG(fmt, ...)                  \
    if (GLOBAL_LOG_LEVEL >= LOG_LEVEL_DEBUG) \
    printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)                  \
    if (GLOBAL_LOG_LEVEL >= LOG_LEVEL_INFO) \
    printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)                  \
    if (GLOBAL_LOG_LEVEL >= LOG_LEVEL_WARN) \
    printf("[WARN] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)                  \
    if (GLOBAL_LOG_LEVEL >= LOG_LEVEL_ERROR) \
    printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
#endif // LOG_H
