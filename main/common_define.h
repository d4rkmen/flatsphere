/**
 * @file common_define.h
 * @brief Common defines for the project using FreeRTOS and ESP-IDF
 * @author d4rkmen
 * @license Apache License 2.0
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_timer.h"

#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define millis() (esp_timer_get_time() / 1000)