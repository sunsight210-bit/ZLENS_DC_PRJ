// Test/mocks/mock_freertos.cpp
#include "mock_freertos.hpp"
#include <queue>
#include <vector>
#include <cstring>

struct MockQueue {
    std::queue<std::vector<uint8_t>> items;
    uint32_t itemSize;
};

static std::vector<MockQueue*> s_queues;

QueueHandle_t xQueueCreate(uint32_t length, uint32_t itemSize) {
    (void)length;
    auto* q = new MockQueue();
    q->itemSize = itemSize;
    s_queues.push_back(q);
    return q;
}

BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t) {
    auto* q = static_cast<MockQueue*>(queue);
    std::vector<uint8_t> data(q->itemSize);
    std::memcpy(data.data(), item, q->itemSize);
    q->items.push(std::move(data));
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void* item, TickType_t) {
    auto* q = static_cast<MockQueue*>(queue);
    if (q->items.empty()) return pdFALSE;
    std::memcpy(item, q->items.front().data(), q->itemSize);
    q->items.pop();
    return pdTRUE;
}

BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void* item, BaseType_t*) {
    return xQueueSend(queue, item, 0);
}

void xQueueReset(QueueHandle_t queue) {
    auto* q = static_cast<MockQueue*>(queue);
    while (!q->items.empty()) q->items.pop();
}

uint32_t uxQueueMessagesWaiting(QueueHandle_t queue) {
    auto* q = static_cast<MockQueue*>(queue);
    return static_cast<uint32_t>(q->items.size());
}

void vTaskDelay(TickType_t) {}
void vTaskDelayUntil(TickType_t*, TickType_t) {}

static uint32_t s_notifyValue = 0;
static BaseType_t s_notifyReturn = pdFALSE;

BaseType_t xTaskNotify(TaskHandle_t, uint32_t value, uint32_t) {
    s_notifyValue = value;
    return pdTRUE;
}

BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t* value, TickType_t) {
    if (s_notifyReturn == pdTRUE && value) {
        *value = s_notifyValue;
    }
    return s_notifyReturn;
}

void taskENTER_CRITICAL(void) {}
void taskEXIT_CRITICAL(void) {}

// Test helpers
namespace mock_rtos {
    void set_notify_return(BaseType_t ret, uint32_t value) {
        s_notifyReturn = ret;
        s_notifyValue = value;
    }
}
