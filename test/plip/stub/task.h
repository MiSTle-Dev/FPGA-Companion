#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
int xTaskCreate(void (*f)(void*), const char*, int, void*, int, TaskHandle_t*);
uint32_t ulTaskNotifyTake(int, TickType_t);
int xTaskNotifyGive(TaskHandle_t);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t);
