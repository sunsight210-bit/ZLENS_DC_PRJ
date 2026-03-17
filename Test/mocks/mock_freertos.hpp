// Test/mocks/mock_freertos.hpp
#pragma once
#include <cstdint>

// Minimal FreeRTOS type stubs
typedef void* QueueHandle_t;
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
typedef int32_t BaseType_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  pdTRUE
#define portMAX_DELAY 0xFFFFFFFF

QueueHandle_t xQueueCreate(uint32_t length, uint32_t itemSize);
BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t queue, void* item, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void* item, BaseType_t* woken);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t* prev, TickType_t increment);
BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, uint32_t action);
BaseType_t xTaskNotifyWait(uint32_t clearEntry, uint32_t clearExit, uint32_t* value, TickType_t wait);
void taskENTER_CRITICAL(void);
void taskEXIT_CRITICAL(void);
