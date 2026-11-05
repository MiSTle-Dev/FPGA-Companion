#pragma once
#include <stdint.h>
typedef uint32_t TickType_t; typedef int BaseType_t;
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define configMAX_PRIORITIES 32
