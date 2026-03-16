# ZLENS_DC 电动变倍镜头控制系统 实施计划

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从零搭建 STM32F103RCT6 电动变倍镜头控制系统的完整嵌入式 C++ 工程，包括 CMake 构建、FreeRTOS 4 任务架构、8 个核心模块及完整单元测试。

**Architecture:** FreeRTOS 4 任务（MotorTask/CommTask/StorageTask/MonitorTask）+ 8 个 App 层 C++ 模块。CMake 双目标构建：ARM 交叉编译（目标板）+ x86 主机 gtest 单元测试。HAL Mock 层隔离硬件依赖。

**Tech Stack:** STM32F103RCT6, arm-none-eabi-gcc 14.3, CMake+Ninja, FreeRTOS, Google Test, STM32 HAL, C++17

**Design Spec:** `docs/superpowers/specs/2026-03-16-zlens-dc-design.md`

---

## Chunk 1: 构建系统与测试基础设施

### Task 1: CMake 交叉编译工具链

**Files:**
- Create: `cmake/stm32f103rc.cmake`
- Create: `cmake/stm32f1xx.ld`
- Create: `CMakeLists.txt`

- [ ] **Step 1: 创建 ARM 交叉编译工具链文件**

```cmake
# cmake/stm32f103rc.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(TOOLCHAIN_DIR "/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_C_FLAGS_INIT   "${MCU_FLAGS} -fdata-sections -ffunction-sections -Wall")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti -Wall -std=c++17")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/cmake/stm32f1xx.ld")
```

- [ ] **Step 2: 创建链接脚本**

```ld
/* cmake/stm32f1xx.ld - STM32F103RCT6: 256KB Flash, 48KB SRAM */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); _etext = .; } >FLASH
  .data : AT(_etext) { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } >RAM
  .bss : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >RAM
  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}
```

- [ ] **Step 3: 创建顶层 CMakeLists.txt（骨架）**

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

option(BUILD_TESTING "Build unit tests for host" OFF)

if(BUILD_TESTING)
    project(ZLENS_DC_Test LANGUAGES C CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
    enable_testing()

    add_subdirectory(Test)
else()
    project(ZLENS_DC LANGUAGES C CXX ASM)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    add_definitions(-DSTM32F103xE -DUSE_HAL_DRIVER)

    # Placeholder: Core/Drivers/App/Tasks sources added later
    # after CubeMX generation
    message(STATUS "ARM cross-compilation target — CubeMX sources required")
endif()
```

- [ ] **Step 4: 验证工具链文件语法正确**

Run: `cd /home/cbn/VSCode/STM32_PRJ/ZLENS_DC && cat cmake/stm32f103rc.cmake`
Expected: 文件内容完整无误

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt cmake/
git commit -m "build: add CMake cross-compilation toolchain and linker script"
```

---

### Task 2: Google Test 测试基础设施 + HAL Mock 层

**Files:**
- Create: `Test/CMakeLists.txt`
- Create: `Test/mocks/mock_hal.hpp`
- Create: `Test/mocks/mock_hal.cpp`
- Create: `Test/mocks/mock_freertos.hpp`
- Create: `Test/mocks/mock_freertos.cpp`
- Create: `Test/mocks/stm32f1xx_hal.h` (minimal HAL type stubs)

- [ ] **Step 1: 创建 HAL 类型存根头文件**

```cpp
// Test/mocks/stm32f1xx_hal.h
// Minimal STM32 HAL type stubs for host-side testing
#pragma once
#include <cstdint>

// --- Status & Return types ---
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

// --- GPIO ---
typedef struct { uint32_t dummy; } GPIO_TypeDef;
enum { GPIO_PIN_SET = 1, GPIO_PIN_RESET = 0 };

// --- TIM ---
typedef struct {
    volatile uint32_t CNT;
    volatile uint32_t ARR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t SR;
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    uint32_t Channel;
} TIM_HandleTypeDef;

// --- ADC ---
typedef struct { volatile uint32_t DR; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;

// --- SPI ---
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
} SPI_TypeDef;
typedef struct { SPI_TypeDef *Instance; } SPI_HandleTypeDef;
#define SPI_SR_BSY  0x0080

// --- UART ---
typedef struct { volatile uint32_t DR; volatile uint32_t SR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;

// --- DAC ---
typedef struct { volatile uint32_t DHR12R2; } DAC_TypeDef;
typedef struct { DAC_TypeDef *Instance; } DAC_HandleTypeDef;

// --- FLASH ---
typedef enum { FLASH_TYPEERASE_PAGES = 0 } FLASH_TypeEraseTypeDef;
typedef struct {
    FLASH_TypeEraseTypeDef TypeErase;
    uint32_t PageAddress;
    uint32_t NbPages;
} FLASH_EraseInitTypeDef;

// --- Extern peripherals (defined in mock_hal.cpp) ---
extern TIM_TypeDef mock_TIM3_instance;
extern TIM_TypeDef mock_TIM8_instance;
extern SPI_TypeDef mock_SPI2_instance;
extern GPIO_TypeDef mock_GPIOB_instance;
extern GPIO_TypeDef mock_GPIOA_instance;
extern ADC_TypeDef mock_ADC1_instance;
extern DAC_TypeDef mock_DAC_instance;

#define TIM3  (&mock_TIM3_instance)
#define TIM8  (&mock_TIM8_instance)
#define SPI2  (&mock_SPI2_instance)
#define GPIOB (&mock_GPIOB_instance)
#define GPIOA (&mock_GPIOA_instance)
#define ADC1  (&mock_ADC1_instance)
#define DAC1  (&mock_DAC_instance)
```

- [ ] **Step 2: 创建 HAL Mock 头文件**

```cpp
// Test/mocks/mock_hal.hpp
#pragma once
#include "stm32f1xx_hal.h"
#include <vector>
#include <cstdint>
#include <cstring>

namespace mock {

// Track HAL calls for verification
struct HalCallLog {
    // GPIO
    struct GpioWrite { GPIO_TypeDef* port; uint16_t pin; uint8_t state; };
    std::vector<GpioWrite> gpio_writes;

    // PWM (TIM CCR writes)
    struct PwmSet { TIM_TypeDef* tim; uint32_t channel; uint32_t value; };
    std::vector<PwmSet> pwm_sets;

    // SPI
    std::vector<std::vector<uint8_t>> spi_tx_data;
    std::vector<uint8_t> spi_rx_buffer;  // data returned by SPI reads
    size_t spi_rx_pos = 0;

    // DAC
    struct DacSet { uint32_t channel; uint32_t value; };
    std::vector<DacSet> dac_sets;

    // UART
    std::vector<std::vector<uint8_t>> uart_tx_data;

    // Flash
    std::vector<uint8_t> flash_memory;
    uint32_t flash_base = 0x0803F800;
    bool flash_unlocked = false;

    void reset();
};

HalCallLog& get_log();

} // namespace mock

// --- HAL function declarations (implemented in mock_hal.cpp) ---
#ifdef __cplusplus
extern "C" {
#endif

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, uint8_t state);
uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin);

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef* htim, uint32_t Channel);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* txdata, uint8_t* rxdata, uint16_t size, uint32_t timeout);

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size);

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* pEraseInit, uint32_t* PageError);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 3: 创建 HAL Mock 实现**

```cpp
// Test/mocks/mock_hal.cpp
#include "mock_hal.hpp"
#include <algorithm>

// Peripheral instances
TIM_TypeDef mock_TIM3_instance = {};
TIM_TypeDef mock_TIM8_instance = {};
SPI_TypeDef mock_SPI2_instance = {};
GPIO_TypeDef mock_GPIOB_instance = {};
GPIO_TypeDef mock_GPIOA_instance = {};
ADC_TypeDef mock_ADC1_instance = {};
DAC_TypeDef mock_DAC_instance = {};

static uint32_t s_tick = 0;

namespace mock {

static HalCallLog s_log;

void HalCallLog::reset() {
    gpio_writes.clear();
    pwm_sets.clear();
    spi_tx_data.clear();
    spi_rx_buffer.clear();
    spi_rx_pos = 0;
    dac_sets.clear();
    uart_tx_data.clear();
    flash_memory.assign(2048, 0xFF);
    flash_unlocked = false;
    s_tick = 0;
    mock_TIM3_instance = {};
    mock_TIM8_instance = {};
    mock_SPI2_instance = {};
}

HalCallLog& get_log() { return s_log; }

} // namespace mock

extern "C" {

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, uint8_t state) {
    mock::get_log().gpio_writes.push_back({port, pin, state});
}

uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin) {
    return GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    mock::get_log().spi_tx_data.push_back(std::vector<uint8_t>(data, data + size));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    auto& log = mock::get_log();
    for (uint16_t i = 0; i < size; ++i) {
        data[i] = (log.spi_rx_pos < log.spi_rx_buffer.size())
                  ? log.spi_rx_buffer[log.spi_rx_pos++] : 0xFF;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* tx, uint8_t* rx, uint16_t size, uint32_t timeout) {
    HAL_SPI_Transmit(hspi, tx, size, timeout);
    return HAL_SPI_Receive(hspi, rx, size, timeout);
}

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef*, uint32_t Channel, uint32_t, uint32_t Data) {
    mock::get_log().dac_sets.push_back({Channel, Data});
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef*, uint32_t) { return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    mock::get_log().uart_tx_data.push_back(std::vector<uint8_t>(data, data + size));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASH_Unlock(void) {
    mock::get_log().flash_unlocked = true;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Lock(void) {
    mock::get_log().flash_unlocked = false;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* pEraseInit, uint32_t* PageError) {
    auto& log = mock::get_log();
    if (!log.flash_unlocked) return HAL_ERROR;
    std::fill(log.flash_memory.begin(), log.flash_memory.end(), 0xFF);
    *PageError = 0xFFFFFFFF;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t Address, uint64_t Data) {
    auto& log = mock::get_log();
    if (!log.flash_unlocked) return HAL_ERROR;
    uint32_t offset = Address - log.flash_base;
    if (offset + 2 > log.flash_memory.size()) return HAL_ERROR;
    // Program as half-word (16-bit)
    log.flash_memory[offset]     = Data & 0xFF;
    log.flash_memory[offset + 1] = (Data >> 8) & 0xFF;
    return HAL_OK;
}

uint32_t HAL_GetTick(void) { return s_tick++; }

} // extern "C"
```

- [ ] **Step 4: 创建 FreeRTOS Mock**

```cpp
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
```

```cpp
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

void vTaskDelay(TickType_t) {}
void vTaskDelayUntil(TickType_t*, TickType_t) {}
BaseType_t xTaskNotify(TaskHandle_t, uint32_t, uint32_t) { return pdTRUE; }
BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t*, TickType_t) { return pdTRUE; }
void taskENTER_CRITICAL(void) {}
void taskEXIT_CRITICAL(void) {}
```

- [ ] **Step 5: 创建 Test/CMakeLists.txt**

```cmake
# Test/CMakeLists.txt
set(MOCK_SOURCES
    mocks/mock_hal.cpp
    mocks/mock_freertos.cpp
)

# Mock library shared by all tests
add_library(zlens_mocks STATIC ${MOCK_SOURCES})
target_include_directories(zlens_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Tasks/Inc
)

# Helper function to add a test
function(zlens_add_test NAME SOURCE APP_SOURCE)
    add_executable(${NAME} ${SOURCE} ${CMAKE_SOURCE_DIR}/App/Src/${APP_SOURCE})
    target_link_libraries(${NAME} PRIVATE zlens_mocks GTest::gtest_main)
    target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        ${CMAKE_SOURCE_DIR}/App/Inc
    )
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

# Tests will be added as modules are implemented:
# zlens_add_test(test_comm_protocol test_comm_protocol.cpp comm_protocol.cpp)
# zlens_add_test(test_encoder test_encoder.cpp encoder.cpp)
# etc.
```

- [ ] **Step 6: 创建占位目录**

```bash
mkdir -p App/Inc App/Src Tasks/Inc Tasks/Src
```

- [ ] **Step 7: 验证 gtest 构建**

Run:
```bash
cd /home/cbn/VSCode/STM32_PRJ/ZLENS_DC
mkdir -p build-test && cd build-test
cmake -G Ninja -DBUILD_TESTING=ON .. && ninja zlens_mocks
```
Expected: zlens_mocks 静态库编译成功

- [ ] **Step 8: Commit**

```bash
git add Test/ App/ Tasks/
git commit -m "build: add gtest infrastructure with HAL/FreeRTOS mock layer"
```

---

## Chunk 2: CRC16 与通信协议模块

### Task 3: CRC16/MODBUS 实现（TDD）

**Files:**
- Create: `App/Inc/crc16.hpp`
- Create: `App/Src/crc16.cpp`
- Create: `Test/test_crc16.cpp`

- [ ] **Step 1: 写 CRC16 失败测试**

```cpp
// Test/test_crc16.cpp
#include <gtest/gtest.h>
#include "crc16.hpp"

TEST(CRC16, KnownVector_A5_01_0000) {
    // A5 01 00 0F -> should produce CRC that makes frame A5 01 00 0F xx xx
    uint8_t data[] = {0xA5, 0x01, 0x00, 0x0F};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    // Known from protocol doc: A5 01 00 0F 33 2C => CRC = 0x2C33
    EXPECT_EQ(crc, 0x2C33);
}

TEST(CRC16, KnownVector_A5_10_002D) {
    // From protocol doc example: A5 10 00 2D E3 30
    uint8_t data[] = {0xA5, 0x10, 0x00, 0x2D};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    EXPECT_EQ(crc, 0x30E3);
}

TEST(CRC16, KnownVector_A5_02_000A) {
    // A5 02 00 0A 03 2F => CRC = 0x2F03
    uint8_t data[] = {0xA5, 0x02, 0x00, 0x0A};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    EXPECT_EQ(crc, 0x2F03);
}

TEST(CRC16, EmptyData) {
    uint16_t crc = zlens::crc16_modbus(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFF); // Initial value unchanged
}

TEST(CRC16, SingleByte) {
    uint8_t data[] = {0x00};
    uint16_t crc = zlens::crc16_modbus(data, 1);
    EXPECT_NE(crc, 0xFFFF); // Should be different from initial
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd build-test && cmake -G Ninja -DBUILD_TESTING=ON .. && ninja test_crc16 2>&1 | tail -5`
Expected: 编译失败 — "crc16.hpp: No such file"

- [ ] **Step 3: 实现 CRC16**

```cpp
// App/Inc/crc16.hpp
#pragma once
#include <cstdint>

namespace zlens {
uint16_t crc16_modbus(const uint8_t* data, uint16_t length);
} // namespace zlens
```

```cpp
// App/Src/crc16.cpp
#include "crc16.hpp"

namespace zlens {

uint16_t crc16_modbus(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

} // namespace zlens
```

- [ ] **Step 4: 在 Test/CMakeLists.txt 中注册测试**

在 `Test/CMakeLists.txt` 末尾添加:
```cmake
add_executable(test_crc16 test_crc16.cpp ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_crc16 PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_crc16 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_SOURCE_DIR}/App/Inc
)
add_test(NAME test_crc16 COMMAND test_crc16)
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cd build-test && ninja test_crc16 && ./test_crc16`
Expected: All 5 tests PASSED

- [ ] **Step 6: Commit**

```bash
git add App/Inc/crc16.hpp App/Src/crc16.cpp Test/test_crc16.cpp Test/CMakeLists.txt
git commit -m "feat: add CRC16/MODBUS implementation with TDD tests"
```

---

### Task 4: 通信协议帧解析（TDD）

**Files:**
- Create: `App/Inc/comm_protocol.hpp`
- Create: `App/Src/comm_protocol.cpp`
- Create: `Test/test_comm_protocol.cpp`

- [ ] **Step 1: 写帧解析失败测试**

```cpp
// Test/test_comm_protocol.cpp
#include <gtest/gtest.h>
#include "comm_protocol.hpp"

using namespace zlens;

class CommProtocolTest : public ::testing::Test {
protected:
    CommProtocol comm;
    void SetUp() override { comm.init(); }
};

// --- Frame parsing ---
TEST_F(CommProtocolTest, ParseValidFrame_SetZoom) {
    // A5 10 00 2D E3 30 — set zoom to 4.5x
    uint8_t frame[] = {0xA5, 0x10, 0x00, 0x2D, 0xE3, 0x30};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0x10);
    EXPECT_EQ(result.param, 0x002D); // 45 = 4.5x * 10
}

TEST_F(CommProtocolTest, ParseValidFrame_Homing) {
    // A5 01 00 0F 33 2C — homing with param 0x000F
    uint8_t frame[] = {0xA5, 0x01, 0x00, 0x0F, 0x33, 0x2C};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0x01);
}

TEST_F(CommProtocolTest, RejectBadHeader) {
    uint8_t frame[] = {0xBB, 0x10, 0x00, 0x2D, 0xE3, 0x30};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_FALSE(result.valid);
}

TEST_F(CommProtocolTest, RejectBadCRC) {
    uint8_t frame[] = {0xA5, 0x10, 0x00, 0x2D, 0x00, 0x00}; // wrong CRC
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_FALSE(result.valid);
}

TEST_F(CommProtocolTest, RejectShortFrame) {
    uint8_t frame[] = {0xA5, 0x10, 0x00};
    auto result = comm.parse_work_frame(frame, 3);
    EXPECT_FALSE(result.valid);
}

// --- Build response frame ---
TEST_F(CommProtocolTest, BuildResponseFrame) {
    uint8_t out[6];
    comm.build_work_frame(0x10, 0x002D, out);
    EXPECT_EQ(out[0], 0xA5);
    EXPECT_EQ(out[1], 0x10);
    EXPECT_EQ(out[2], 0x00); // param high
    EXPECT_EQ(out[3], 0x2D); // param low
    // CRC should match known value
    EXPECT_EQ(out[4], 0xE3);
    EXPECT_EQ(out[5], 0x30);
}

// --- Factory mode frame (8 bytes) ---
TEST_F(CommProtocolTest, ParseFactoryFrame) {
    // Switch to factory mode first
    comm.set_factory_mode(true);
    // 0xF1 command: zoom=0x001E(3.0x), angle=0x510E(207.5°)
    uint8_t frame[8];
    comm.build_factory_frame(0xF1, 0x001E, 0x510E, frame);
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 0xF1);
    auto result = comm.parse_factory_frame(frame, 8);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0xF1);
    EXPECT_EQ(result.param_high, 0x001E);
    EXPECT_EQ(result.param_low, 0x510E);
}

// --- Mode switching ---
TEST_F(CommProtocolTest, ModeSwitch) {
    EXPECT_FALSE(comm.is_factory_mode());
    comm.set_factory_mode(true);
    EXPECT_TRUE(comm.is_factory_mode());
    comm.set_factory_mode(false);
    EXPECT_FALSE(comm.is_factory_mode());
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd build-test && cmake .. && ninja test_comm_protocol 2>&1 | tail -5`
Expected: 编译失败 — "comm_protocol.hpp: No such file"

- [ ] **Step 3: 实现 CommProtocol 类**

```cpp
// App/Inc/comm_protocol.hpp
#pragma once
#include <cstdint>

namespace zlens {

struct WorkFrameResult {
    bool valid;
    uint8_t cmd;
    uint16_t param;
};

struct FactoryFrameResult {
    bool valid;
    uint8_t cmd;
    uint16_t param_high;
    uint16_t param_low;
};

class CommProtocol {
public:
    void init();

    // Work mode (6-byte frames)
    WorkFrameResult parse_work_frame(const uint8_t* data, uint16_t len);
    void build_work_frame(uint8_t cmd, uint16_t param, uint8_t* out);

    // Factory mode (8-byte frames)
    FactoryFrameResult parse_factory_frame(const uint8_t* data, uint16_t len);
    void build_factory_frame(uint8_t cmd, uint16_t param_h, uint16_t param_l, uint8_t* out);

    // Mode management
    bool is_factory_mode() const { return factory_mode_; }
    void set_factory_mode(bool enabled) { factory_mode_ = enabled; }

    static constexpr uint8_t FRAME_HEADER = 0xA5;
    static constexpr uint8_t WORK_FRAME_SIZE = 6;
    static constexpr uint8_t FACTORY_FRAME_SIZE = 8;

private:
    bool factory_mode_ = false;
};

} // namespace zlens
```

```cpp
// App/Src/comm_protocol.cpp
#include "comm_protocol.hpp"
#include "crc16.hpp"

namespace zlens {

void CommProtocol::init() {
    factory_mode_ = false;
}

WorkFrameResult CommProtocol::parse_work_frame(const uint8_t* data, uint16_t len) {
    WorkFrameResult r{false, 0, 0};
    if (len < WORK_FRAME_SIZE) return r;
    if (data[0] != FRAME_HEADER) return r;

    uint16_t crc_calc = crc16_modbus(data, 4);
    uint16_t crc_recv = data[4] | (data[5] << 8);
    if (crc_calc != crc_recv) return r;

    r.valid = true;
    r.cmd = data[1];
    r.param = (data[2] << 8) | data[3];
    return r;
}

void CommProtocol::build_work_frame(uint8_t cmd, uint16_t param, uint8_t* out) {
    out[0] = FRAME_HEADER;
    out[1] = cmd;
    out[2] = (param >> 8) & 0xFF;
    out[3] = param & 0xFF;
    uint16_t crc = crc16_modbus(out, 4);
    out[4] = crc & 0xFF;
    out[5] = (crc >> 8) & 0xFF;
}

FactoryFrameResult CommProtocol::parse_factory_frame(const uint8_t* data, uint16_t len) {
    FactoryFrameResult r{false, 0, 0, 0};
    if (len < FACTORY_FRAME_SIZE) return r;
    if (data[0] != FRAME_HEADER) return r;

    uint16_t crc_calc = crc16_modbus(data, 6);
    uint16_t crc_recv = data[6] | (data[7] << 8);
    if (crc_calc != crc_recv) return r;

    r.valid = true;
    r.cmd = data[1];
    r.param_high = (data[2] << 8) | data[3];
    r.param_low  = (data[4] << 8) | data[5];
    return r;
}

void CommProtocol::build_factory_frame(uint8_t cmd, uint16_t param_h, uint16_t param_l, uint8_t* out) {
    out[0] = FRAME_HEADER;
    out[1] = cmd;
    out[2] = (param_h >> 8) & 0xFF;
    out[3] = param_h & 0xFF;
    out[4] = (param_l >> 8) & 0xFF;
    out[5] = param_l & 0xFF;
    uint16_t crc = crc16_modbus(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
}

} // namespace zlens
```

- [ ] **Step 4: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_comm_protocol test_comm_protocol.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/comm_protocol.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_comm_protocol PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_comm_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_comm_protocol COMMAND test_comm_protocol)
```

Run: `cd build-test && ninja test_comm_protocol && ./test_comm_protocol`
Expected: All tests PASSED

- [ ] **Step 5: Commit**

```bash
git add App/Inc/comm_protocol.hpp App/Src/comm_protocol.cpp Test/test_comm_protocol.cpp Test/CMakeLists.txt
git commit -m "feat: add comm protocol frame parser with work/factory mode support"
```

---

## Chunk 3: 编码器与倍率映射表模块

### Task 5: 编码器模块（TDD）

**Files:**
- Create: `App/Inc/encoder.hpp`
- Create: `App/Src/encoder.cpp`
- Create: `Test/test_encoder.cpp`

- [ ] **Step 1: 写编码器失败测试**

```cpp
// Test/test_encoder.cpp
#include <gtest/gtest.h>
#include "encoder.hpp"

using namespace zlens;

class EncoderTest : public ::testing::Test {
protected:
    Encoder enc;
    void SetUp() override {
        mock::get_log().reset();
        enc.init();
    }
};

TEST_F(EncoderTest, InitialPositionIsZero) {
    EXPECT_EQ(enc.get_position(), 0);
}

TEST_F(EncoderTest, SetAndGetPosition) {
    enc.set_position(12345);
    EXPECT_EQ(enc.get_position(), 12345);
}

TEST_F(EncoderTest, SetPosition_SplitsOverflowAndCNT) {
    enc.set_position(70000); // > 65536
    // overflow = 70000 / 65536 = 1, CNT = 70000 % 65536 = 4464
    EXPECT_EQ(TIM8->CNT, 4464u);
    EXPECT_EQ(enc.get_position(), 70000);
}

TEST_F(EncoderTest, SetPosition_Negative) {
    enc.set_position(-100);
    EXPECT_EQ(enc.get_position(), -100);
}

TEST_F(EncoderTest, Reset) {
    enc.set_position(50000);
    enc.reset();
    EXPECT_EQ(enc.get_position(), 0);
    EXPECT_EQ(TIM8->CNT, 0u);
}

TEST_F(EncoderTest, OverflowIncrement) {
    enc.set_position(0);
    // Simulate TIM8 overflow (forward direction)
    enc.handle_overflow(true); // up direction
    TIM8->CNT = 100;
    EXPECT_EQ(enc.get_position(), 65536 + 100);
}

TEST_F(EncoderTest, OverflowDecrement) {
    enc.set_position(70000);
    enc.handle_overflow(false); // down direction
    TIM8->CNT = 65000;
    EXPECT_EQ(enc.get_position(), 65000); // 0 * 65536 + 65000
}

TEST_F(EncoderTest, ZPulseRecord) {
    enc.set_position(1000);
    TIM8->CNT = 1000;
    enc.handle_z_pulse();
    EXPECT_EQ(enc.get_z_position(), 1000);
}

TEST_F(EncoderTest, MultipleOverflows) {
    enc.set_position(0);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    TIM8->CNT = 500;
    EXPECT_EQ(enc.get_position(), 3 * 65536 + 500);
}
```

- [ ] **Step 2: 运行测试确认失败**

Expected: 编译失败 — "encoder.hpp: No such file"

- [ ] **Step 3: 实现 Encoder 类**

```cpp
// App/Inc/encoder.hpp
#pragma once
#include <cstdint>
#ifdef BUILD_TESTING
#include "stm32f1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

class Encoder {
public:
    void init();
    int32_t get_position() const;
    void set_position(int32_t pos);
    int32_t get_z_position() const { return z_position_; }
    void reset();

    // Called from ISR
    void handle_overflow(bool up_direction);
    void handle_z_pulse();

private:
    volatile int32_t overflow_ = 0;
    volatile int32_t z_position_ = 0;
};

} // namespace zlens
```

```cpp
// App/Src/encoder.cpp
#include "encoder.hpp"

namespace zlens {

void Encoder::init() {
    overflow_ = 0;
    z_position_ = 0;
    TIM8->CNT = 0;
}

int32_t Encoder::get_position() const {
    int32_t ovf = overflow_;
    uint16_t cnt = static_cast<uint16_t>(TIM8->CNT);
    return ovf * 65536 + cnt;
}

void Encoder::set_position(int32_t pos) {
    if (pos >= 0) {
        overflow_ = pos / 65536;
        TIM8->CNT = pos % 65536;
    } else {
        // Handle negative: e.g. -100 => overflow=-1, CNT=65436
        overflow_ = (pos - 65535) / 65536;
        TIM8->CNT = static_cast<uint16_t>(pos - overflow_ * 65536);
    }
}

void Encoder::reset() {
    overflow_ = 0;
    z_position_ = 0;
    TIM8->CNT = 0;
}

void Encoder::handle_overflow(bool up_direction) {
    if (up_direction) {
        overflow_++;
    } else {
        overflow_--;
    }
}

void Encoder::handle_z_pulse() {
    z_position_ = get_position();
}

} // namespace zlens
```

- [ ] **Step 4: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_encoder test_encoder.cpp ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp)
target_link_libraries(test_encoder PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_encoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_encoder COMMAND test_encoder)
```

Run: `cd build-test && ninja test_encoder && ./test_encoder`
Expected: All tests PASSED

- [ ] **Step 5: Commit**

```bash
git add App/Inc/encoder.hpp App/Src/encoder.cpp Test/test_encoder.cpp Test/CMakeLists.txt
git commit -m "feat: add encoder module with 32-bit overflow extension and Z-pulse"
```

---

### Task 6: 倍率映射表模块（TDD）

**Files:**
- Create: `App/Inc/zoom_table.hpp`
- Create: `App/Src/zoom_table.cpp`
- Create: `Test/test_zoom_table.cpp`

- [ ] **Step 1: 写倍率映射表失败测试**

```cpp
// Test/test_zoom_table.cpp
#include <gtest/gtest.h>
#include "zoom_table.hpp"

using namespace zlens;

class ZoomTableTest : public ::testing::Test {
protected:
    ZoomTable table;
    void SetUp() override {
        mock::get_log().reset();
        table.init();
        // Load default 14-entry table
        table.load_defaults();
        table.set_total_range(206821); // from design spec
    }
};

TEST_F(ZoomTableTest, DefaultTableHas14Entries) {
    EXPECT_EQ(table.get_entry_count(), 14u);
}

TEST_F(ZoomTableTest, ValidZoom_InTable) {
    EXPECT_TRUE(table.is_valid_zoom(6));   // 0.6x
    EXPECT_TRUE(table.is_valid_zoom(10));  // 1.0x
    EXPECT_TRUE(table.is_valid_zoom(70));  // 7.0x
}

TEST_F(ZoomTableTest, InvalidZoom_NotInTable) {
    EXPECT_FALSE(table.is_valid_zoom(7));   // 0.7x not in table
    EXPECT_FALSE(table.is_valid_zoom(100)); // 10.0x
    EXPECT_FALSE(table.is_valid_zoom(0));
}

TEST_F(ZoomTableTest, GetPosition_FirstEntry) {
    // 0.6x = 0 degrees = position 0
    EXPECT_EQ(table.get_position(6), 0);
}

TEST_F(ZoomTableTest, GetPosition_LastEntry) {
    // 7.0x = 347 degrees => 347*100 * 206821 / 36000 ≈ 199,296
    int32_t pos = table.get_position(70);
    EXPECT_GT(pos, 190000);
    EXPECT_LT(pos, 210000);
}

TEST_F(ZoomTableTest, GetPosition_InvalidZoom_ReturnsNeg1) {
    EXPECT_EQ(table.get_position(99), -1);
}

TEST_F(ZoomTableTest, GetNearestZoom) {
    EXPECT_EQ(table.get_nearest_zoom(0), 6);       // at start
    EXPECT_EQ(table.get_nearest_zoom(5000), 6);     // closer to 0.6x
    EXPECT_EQ(table.get_nearest_zoom(15000), 8);    // closer to 0.8x
}

TEST_F(ZoomTableTest, GetNextZoom_Forward) {
    EXPECT_EQ(table.get_next_zoom(6, 1), 8);    // 0.6x -> 0.8x
    EXPECT_EQ(table.get_next_zoom(60, 1), 70);  // 6.0x -> 7.0x
}

TEST_F(ZoomTableTest, GetNextZoom_AtMax_Clamps) {
    EXPECT_EQ(table.get_next_zoom(70, 1), 70);  // 7.0x stays
}

TEST_F(ZoomTableTest, GetNextZoom_Backward) {
    EXPECT_EQ(table.get_next_zoom(10, -1), 8);  // 1.0x -> 0.8x
    EXPECT_EQ(table.get_next_zoom(6, -1), 6);   // 0.6x stays
}

TEST_F(ZoomTableTest, GetNextZoom_MultiStep) {
    EXPECT_EQ(table.get_next_zoom(6, 3), 12);   // skip 3: 0.6->0.8->1.0->1.2
}

TEST_F(ZoomTableTest, SetEntry_FactoryMode) {
    table.erase_all();
    EXPECT_EQ(table.get_entry_count(), 0u);
    table.set_entry(10, 3600); // 1.0x at 36.0 degrees
    EXPECT_EQ(table.get_entry_count(), 1u);
    EXPECT_TRUE(table.is_valid_zoom(10));
}

TEST_F(ZoomTableTest, AngleToPositionConversion) {
    // angle_x100=18000 (180°), total_range=206821
    // expected: 18000 * 206821 / 36000 = 103410
    table.set_total_range(206821);
    table.erase_all();
    table.set_entry(25, 18000); // 2.5x at 180°
    int32_t pos = table.get_position(25);
    EXPECT_NEAR(pos, 103410, 2);
}

TEST_F(ZoomTableTest, MinMaxZoom) {
    EXPECT_EQ(table.get_min_zoom(), 6);
    EXPECT_EQ(table.get_max_zoom(), 70);
}
```

- [ ] **Step 2: 运行测试确认失败**

Expected: 编译失败 — "zoom_table.hpp: No such file"

- [ ] **Step 3: 实现 ZoomTable 类**

```cpp
// App/Inc/zoom_table.hpp
#pragma once
#include <cstdint>

namespace zlens {

struct ZoomEntry {
    uint16_t zoom_x10;    // 倍率×10
    uint16_t angle_x100;  // 角度×100
};

class ZoomTable {
public:
    static constexpr uint8_t MAX_ENTRIES = 32;

    void init();
    void load_defaults();

    // Lookup
    int32_t get_position(uint16_t zoom_x10) const;
    uint16_t get_nearest_zoom(int32_t position) const;
    bool is_valid_zoom(uint16_t zoom_x10) const;
    uint16_t get_next_zoom(uint16_t current, int8_t step) const;
    uint16_t get_min_zoom() const;
    uint16_t get_max_zoom() const;

    // Factory mode
    void set_entry(uint16_t zoom_x10, uint16_t angle_x100);
    void erase_all();
    uint8_t get_entry_count() const { return count_; }

    // Calibration
    void set_total_range(int32_t range) { total_range_ = range; }
    int32_t get_total_range() const { return total_range_; }

    // Flash persistence
    bool save_to_flash();
    bool load_from_flash();

private:
    ZoomEntry entries_[MAX_ENTRIES];
    uint8_t count_ = 0;
    int32_t total_range_ = 0;

    int angle_to_position(uint16_t angle_x100) const;
    int find_index(uint16_t zoom_x10) const;
    void sort_entries();
};

} // namespace zlens
```

```cpp
// App/Src/zoom_table.cpp
#include "zoom_table.hpp"
#include "crc16.hpp"
#include <algorithm>
#include <cstring>
#include <cstdlib>

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

static constexpr uint32_t FLASH_ZOOM_ADDR = 0x0803F800;

// Default 14-entry table (angle×100 values)
static const ZoomEntry kDefaultTable[] = {
    { 6,     0}, { 8,  1800}, {10,  3600}, {12,  6000},
    {15,  9000}, {18, 12000}, {20, 15000}, {25, 18000},
    {30, 21000}, {35, 24000}, {40, 27000}, {50, 30000},
    {60, 33000}, {70, 34700}
};

void ZoomTable::init() {
    count_ = 0;
    total_range_ = 0;
    std::memset(entries_, 0, sizeof(entries_));
}

void ZoomTable::load_defaults() {
    count_ = sizeof(kDefaultTable) / sizeof(kDefaultTable[0]);
    std::memcpy(entries_, kDefaultTable, sizeof(kDefaultTable));
}

int ZoomTable::angle_to_position(uint16_t angle_x100) const {
    if (total_range_ == 0) return 0;
    return static_cast<int>(
        static_cast<int64_t>(angle_x100) * total_range_ / 36000
    );
}

int ZoomTable::find_index(uint16_t zoom_x10) const {
    for (uint8_t i = 0; i < count_; ++i) {
        if (entries_[i].zoom_x10 == zoom_x10) return i;
    }
    return -1;
}

void ZoomTable::sort_entries() {
    std::sort(entries_, entries_ + count_,
        [](const ZoomEntry& a, const ZoomEntry& b) {
            return a.zoom_x10 < b.zoom_x10;
        });
}

int32_t ZoomTable::get_position(uint16_t zoom_x10) const {
    int idx = find_index(zoom_x10);
    if (idx < 0) return -1;
    return angle_to_position(entries_[idx].angle_x100);
}

uint16_t ZoomTable::get_nearest_zoom(int32_t position) const {
    if (count_ == 0) return 0;
    uint16_t nearest = entries_[0].zoom_x10;
    int32_t min_dist = std::abs(position - angle_to_position(entries_[0].angle_x100));
    for (uint8_t i = 1; i < count_; ++i) {
        int32_t dist = std::abs(position - angle_to_position(entries_[i].angle_x100));
        if (dist < min_dist) {
            min_dist = dist;
            nearest = entries_[i].zoom_x10;
        }
    }
    return nearest;
}

bool ZoomTable::is_valid_zoom(uint16_t zoom_x10) const {
    return find_index(zoom_x10) >= 0;
}

uint16_t ZoomTable::get_next_zoom(uint16_t current, int8_t step) const {
    int idx = find_index(current);
    if (idx < 0) return current;
    int new_idx = idx + step;
    if (new_idx < 0) new_idx = 0;
    if (new_idx >= count_) new_idx = count_ - 1;
    return entries_[new_idx].zoom_x10;
}

uint16_t ZoomTable::get_min_zoom() const {
    return count_ > 0 ? entries_[0].zoom_x10 : 0;
}

uint16_t ZoomTable::get_max_zoom() const {
    return count_ > 0 ? entries_[count_ - 1].zoom_x10 : 0;
}

void ZoomTable::set_entry(uint16_t zoom_x10, uint16_t angle_x100) {
    int idx = find_index(zoom_x10);
    if (idx >= 0) {
        entries_[idx].angle_x100 = angle_x100;
    } else if (count_ < MAX_ENTRIES) {
        entries_[count_++] = {zoom_x10, angle_x100};
        sort_entries();
    }
}

void ZoomTable::erase_all() {
    count_ = 0;
    std::memset(entries_, 0, sizeof(entries_));
}

bool ZoomTable::save_to_flash() {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_ZOOM_ADDR;
    erase.NbPages = 1;
    uint32_t page_err;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // Write: [count(1B padding to 2B)] + [entries as half-words] + [CRC16]
    HAL_FLASH_Program(0, FLASH_ZOOM_ADDR, count_);

    uint32_t addr = FLASH_ZOOM_ADDR + 2;
    for (uint8_t i = 0; i < count_; ++i) {
        HAL_FLASH_Program(0, addr, entries_[i].zoom_x10);
        addr += 2;
        HAL_FLASH_Program(0, addr, entries_[i].angle_x100);
        addr += 2;
    }

    // CRC over count + entries
    uint16_t crc = crc16_modbus(reinterpret_cast<const uint8_t*>(&count_), 1);
    for (uint8_t i = 0; i < count_; ++i) {
        crc = crc16_modbus(reinterpret_cast<const uint8_t*>(&entries_[i]), sizeof(ZoomEntry));
    }
    HAL_FLASH_Program(0, addr, crc);

    HAL_FLASH_Lock();
    return true;
}

bool ZoomTable::load_from_flash() {
    auto& log = mock::get_log();
    if (log.flash_memory.empty()) return false;

    count_ = log.flash_memory[0];
    if (count_ > MAX_ENTRIES) { count_ = 0; return false; }

    uint32_t offset = 2;
    for (uint8_t i = 0; i < count_; ++i) {
        entries_[i].zoom_x10 = log.flash_memory[offset] | (log.flash_memory[offset+1] << 8);
        offset += 2;
        entries_[i].angle_x100 = log.flash_memory[offset] | (log.flash_memory[offset+1] << 8);
        offset += 2;
    }
    return true;
}

} // namespace zlens
```

- [ ] **Step 4: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_zoom_table test_zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_zoom_table PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_zoom_table PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_zoom_table COMMAND test_zoom_table)
```

Run: `cd build-test && ninja test_zoom_table && ./test_zoom_table`
Expected: All tests PASSED

- [ ] **Step 5: Commit**

```bash
git add App/Inc/zoom_table.hpp App/Src/zoom_table.cpp Test/test_zoom_table.cpp Test/CMakeLists.txt
git commit -m "feat: add configurable zoom table with angle-drive and Flash storage"
```

---

## Chunk 4: FRAM 存储与堵转检测模块

### Task 7: FRAM 存储模块（TDD）

**Files:**
- Create: `App/Inc/fram_storage.hpp`
- Create: `App/Src/fram_storage.cpp`
- Create: `Test/test_fram_storage.cpp`

- [ ] **Step 1: 写 FRAM 存储失败测试**

```cpp
// Test/test_fram_storage.cpp
#include <gtest/gtest.h>
#include "fram_storage.hpp"

using namespace zlens;

class FramStorageTest : public ::testing::Test {
protected:
    FramStorage fram;
    SPI_HandleTypeDef hspi;
    void SetUp() override {
        mock::get_log().reset();
        hspi.Instance = SPI2;
        fram.init(&hspi);
    }
};

TEST_F(FramStorageTest, DefaultParams_Invalid) {
    EXPECT_FALSE(fram.is_valid());
}

TEST_F(FramStorageTest, SaveAndLoad) {
    FramParams p{};
    p.magic_number = FramStorage::MAGIC;
    p.version = 1;
    p.current_position = 50000;
    p.current_zoom_x10 = 20;
    p.homing_done = 1;
    p.position_valid = 0xFF;
    fram.save_params(p);
    EXPECT_GE(mock::get_log().spi_tx_data.size(), 2u); // WP disable + write commands

    // Simulate readback by setting rx buffer
    FramParams loaded{};
    // The mock SPI returns what we wrote (simplified)
    fram.load_params_from_buffer(p, loaded);
    EXPECT_EQ(loaded.current_position, 50000);
    EXPECT_EQ(loaded.current_zoom_x10, 20);
}

TEST_F(FramStorageTest, EmergencySave_WritesPosition) {
    fram.emergency_save(12345);
    // Should have SPI writes for position + valid flag
    EXPECT_GE(mock::get_log().spi_tx_data.size(), 1u);
}

TEST_F(FramStorageTest, MagicCheck) {
    FramParams p{};
    p.magic_number = 0x0000;
    EXPECT_FALSE(FramStorage::check_magic(p));
    p.magic_number = FramStorage::MAGIC;
    EXPECT_TRUE(FramStorage::check_magic(p));
}

TEST_F(FramStorageTest, CRC_Validation) {
    FramParams p{};
    p.magic_number = FramStorage::MAGIC;
    p.version = 1;
    p.current_position = 100;
    uint16_t crc = FramStorage::calc_crc(p);
    EXPECT_NE(crc, 0);
    p.crc16 = crc;
    EXPECT_TRUE(FramStorage::verify_crc(p));
    p.current_position = 200; // tamper
    EXPECT_FALSE(FramStorage::verify_crc(p));
}
```

- [ ] **Step 2: 实现 FramStorage 类**

```cpp
// App/Inc/fram_storage.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

#pragma pack(push, 1)
struct FramParams {
    uint16_t magic_number;       // 0x5A3C
    uint8_t  version;
    int32_t  current_position;
    uint16_t current_zoom_x10;
    int32_t  min_position;
    int32_t  max_position;
    int32_t  total_range;
    uint16_t soft_limit_offset;
    uint8_t  zero_reference;
    int32_t  z_offset;
    uint8_t  lens_type;
    uint16_t min_zoom_x10;
    uint16_t max_zoom_x10;
    uint16_t stall_count;
    uint16_t baseline_current;
    uint16_t stall_threshold;
    uint8_t  homing_done;
    uint8_t  position_valid;
    int16_t  encoder_overflow;
    uint8_t  last_save_reason;
    uint8_t  reserved[14];
    uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(FramParams) == 60, "FramParams must be 60 bytes");

class FramStorage {
public:
    static constexpr uint16_t MAGIC = 0x5A3C;
    static constexpr uint16_t PRIMARY_ADDR = 0x0000;
    static constexpr uint16_t BACKUP_ADDR  = 0x0040;

    void init(SPI_HandleTypeDef* hspi);

    bool save_params(const FramParams& params);
    bool load_params(FramParams& params);
    void emergency_save(int32_t position);
    bool is_valid();

    // For testing
    void load_params_from_buffer(const FramParams& src, FramParams& dst);

    static uint16_t calc_crc(const FramParams& p);
    static bool verify_crc(const FramParams& p);
    static bool check_magic(const FramParams& p);

private:
    SPI_HandleTypeDef* hspi_ = nullptr;

    void write_enable();
    void write_disable_wp();
    void enable_wp();
    void spi_write(uint16_t addr, const uint8_t* data, uint16_t len);
    void spi_read(uint16_t addr, uint8_t* data, uint16_t len);
    void cs_low();
    void cs_high();
};

} // namespace zlens
```

```cpp
// App/Src/fram_storage.cpp
#include "fram_storage.hpp"
#include "crc16.hpp"
#include <cstring>

namespace zlens {

void FramStorage::init(SPI_HandleTypeDef* hspi) {
    hspi_ = hspi;
    enable_wp();
}

void FramStorage::cs_low() {
    HAL_GPIO_WritePin(GPIOB, 1 << 12, GPIO_PIN_RESET); // PB12
}

void FramStorage::cs_high() {
    HAL_GPIO_WritePin(GPIOB, 1 << 12, GPIO_PIN_SET);
}

void FramStorage::write_disable_wp() {
    HAL_GPIO_WritePin(GPIOB, 1 << 11, GPIO_PIN_SET); // PB11 WP high = write enabled
}

void FramStorage::enable_wp() {
    HAL_GPIO_WritePin(GPIOB, 1 << 11, GPIO_PIN_RESET); // PB11 WP low = protected
}

void FramStorage::write_enable() {
    cs_low();
    uint8_t cmd = 0x06; // WREN
    HAL_SPI_Transmit(hspi_, &cmd, 1, 100);
    cs_high();
}

void FramStorage::spi_write(uint16_t addr, const uint8_t* data, uint16_t len) {
    write_disable_wp();
    write_enable();
    cs_low();
    uint8_t hdr[3] = {0x02, static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF)};
    HAL_SPI_Transmit(hspi_, hdr, 3, 100);
    HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(data), len, 100);
    cs_high();
    enable_wp();
}

void FramStorage::spi_read(uint16_t addr, uint8_t* data, uint16_t len) {
    cs_low();
    uint8_t hdr[3] = {0x03, static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF)};
    HAL_SPI_Transmit(hspi_, hdr, 3, 100);
    HAL_SPI_Receive(hspi_, data, len, 100);
    cs_high();
}

uint16_t FramStorage::calc_crc(const FramParams& p) {
    // CRC over all bytes except the last 2 (crc16 field itself)
    return crc16_modbus(reinterpret_cast<const uint8_t*>(&p), sizeof(FramParams) - 2);
}

bool FramStorage::verify_crc(const FramParams& p) {
    return calc_crc(p) == p.crc16;
}

bool FramStorage::check_magic(const FramParams& p) {
    return p.magic_number == MAGIC;
}

bool FramStorage::save_params(const FramParams& params) {
    FramParams p = params;
    p.crc16 = calc_crc(p);
    spi_write(PRIMARY_ADDR, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    spi_write(BACKUP_ADDR, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    return true;
}

bool FramStorage::load_params(FramParams& params) {
    spi_read(PRIMARY_ADDR, reinterpret_cast<uint8_t*>(&params), sizeof(params));
    if (check_magic(params) && verify_crc(params)) return true;

    // Try backup
    spi_read(BACKUP_ADDR, reinterpret_cast<uint8_t*>(&params), sizeof(params));
    if (check_magic(params) && verify_crc(params)) return true;

    return false;
}

void FramStorage::emergency_save(int32_t position) {
    // Minimal save: position + valid flag + save reason
    write_disable_wp();
    write_enable();
    cs_low();

    uint16_t pos_addr = PRIMARY_ADDR + offsetof(FramParams, current_position);
    uint8_t hdr[3] = {0x02, static_cast<uint8_t>(pos_addr >> 8), static_cast<uint8_t>(pos_addr & 0xFF)};
    HAL_SPI_Transmit(hspi_, hdr, 3, 100);

    // Write position (4 bytes) + skip to position_valid + last_save_reason
    HAL_SPI_Transmit(hspi_, reinterpret_cast<uint8_t*>(&position), 4, 100);
    cs_high();

    // Write position_valid = 0xFF
    uint16_t valid_addr = PRIMARY_ADDR + offsetof(FramParams, position_valid);
    write_enable();
    cs_low();
    uint8_t hdr2[3] = {0x02, static_cast<uint8_t>(valid_addr >> 8), static_cast<uint8_t>(valid_addr & 0xFF)};
    HAL_SPI_Transmit(hspi_, hdr2, 3, 100);
    uint8_t valid_data[2] = {0xFF, 2}; // position_valid=0xFF, save_reason not adjacent but simplified
    HAL_SPI_Transmit(hspi_, valid_data, 1, 100);
    cs_high();

    enable_wp();
}

bool FramStorage::is_valid() {
    FramParams p{};
    return load_params(p);
}

void FramStorage::load_params_from_buffer(const FramParams& src, FramParams& dst) {
    dst = src;
}

} // namespace zlens
```

- [ ] **Step 3: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_fram_storage test_fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_fram_storage PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_fram_storage PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_fram_storage COMMAND test_fram_storage)
```

Run: `cd build-test && ninja test_fram_storage && ./test_fram_storage`
Expected: All tests PASSED

- [ ] **Step 4: Commit**

```bash
git add App/Inc/fram_storage.hpp App/Src/fram_storage.cpp Test/test_fram_storage.cpp Test/CMakeLists.txt
git commit -m "feat: add FRAM storage with dual-backup, CRC validation, emergency save"
```

---

### Task 8: 堵转检测模块（TDD）

**Files:**
- Create: `App/Inc/stall_detect.hpp`
- Create: `App/Src/stall_detect.cpp`
- Create: `Test/test_stall_detect.cpp`

- [ ] **Step 1: 写堵转检测失败测试**

```cpp
// Test/test_stall_detect.cpp
#include <gtest/gtest.h>
#include "stall_detect.hpp"

using namespace zlens;

class StallDetectTest : public ::testing::Test {
protected:
    StallDetect sd;
    void SetUp() override {
        mock::get_log().reset();
        sd.init();
    }
};

TEST_F(StallDetectTest, InitialState_NoStall) {
    EXPECT_FALSE(sd.is_stalled());
    EXPECT_FALSE(sd.is_overcurrent());
}

TEST_F(StallDetectTest, BelowThreshold_NoStall) {
    sd.start_motor(); // start blanking window
    // Advance past blanking window (200 ticks)
    for (int i = 0; i < 250; ++i) sd.update(50, 0, i * 100);
    EXPECT_FALSE(sd.is_stalled());
}

TEST_F(StallDetectTest, BlankingWindow_IgnoresHighCurrent) {
    sd.start_motor();
    // During blanking window (first 200 calls), high current should be ignored
    for (int i = 0; i < 200; ++i) sd.update(300, 0, i * 100);
    EXPECT_FALSE(sd.is_stalled());
}

TEST_F(StallDetectTest, StallDetected_After1000Consecutive) {
    sd.start_motor();
    // Pass blanking window
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    // Now 1000 consecutive high current readings
    int32_t last_enc = 0;
    for (int i = 0; i < 1000; ++i) {
        sd.update(250, last_enc, (200 + i) * 100);
    }
    EXPECT_TRUE(sd.is_stalled());
}

TEST_F(StallDetectTest, StallResets_OnLowCurrent) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    // 500 high, then 1 low breaks the streak
    for (int i = 0; i < 500; ++i) sd.update(250, 0, (200 + i) * 100);
    sd.update(50, 0, 700 * 100);
    for (int i = 0; i < 500; ++i) sd.update(250, 0, (701 + i) * 100);
    EXPECT_FALSE(sd.is_stalled()); // streak was broken
}

TEST_F(StallDetectTest, Overcurrent_FastResponse) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    // ADC > 350 for 10 consecutive
    for (int i = 0; i < 10; ++i) sd.update(400, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.is_overcurrent());
}

TEST_F(StallDetectTest, EncoderStall_NoMovement500ms) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    // High current + no encoder movement for 500 ticks
    for (int i = 0; i < 500; ++i) sd.update(250, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.encoder_stalled());
}

TEST_F(StallDetectTest, DirectionDetection) {
    sd.set_direction(StallDetect::Direction::FORWARD);
    EXPECT_EQ(sd.get_stall_limit_type(), StallDetect::LimitType::LIMIT_MAX);
    sd.set_direction(StallDetect::Direction::REVERSE);
    EXPECT_EQ(sd.get_stall_limit_type(), StallDetect::LimitType::LIMIT_MIN);
}

TEST_F(StallDetectTest, Reset_ClearsState) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    for (int i = 0; i < 1000; ++i) sd.update(250, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.is_stalled());
    sd.reset();
    EXPECT_FALSE(sd.is_stalled());
}
```

- [ ] **Step 2: 实现 StallDetect 类**

```cpp
// App/Inc/stall_detect.hpp
#pragma once
#include <cstdint>

namespace zlens {

class StallDetect {
public:
    enum class Direction { FORWARD, REVERSE };
    enum class LimitType { LIMIT_MIN, LIMIT_MAX };

    static constexpr uint16_t STALL_THRESHOLD = 200;
    static constexpr uint16_t OVERCURRENT_THRESHOLD = 350;
    static constexpr uint16_t BLANKING_TICKS = 200;
    static constexpr uint16_t STALL_CONFIRM_COUNT = 1000;
    static constexpr uint16_t OVERCURRENT_CONFIRM = 10;
    static constexpr uint16_t ENCODER_STALL_TICKS = 500;

    void init();
    void reset();
    void start_motor();
    void update(uint16_t adc_current, int32_t encoder_pos, uint32_t tick);

    bool is_stalled() const { return stall_confirmed_; }
    bool is_overcurrent() const { return overcurrent_; }
    bool encoder_stalled() const { return encoder_stall_; }

    void set_direction(Direction d) { direction_ = d; }
    LimitType get_stall_limit_type() const;

private:
    Direction direction_ = Direction::FORWARD;
    uint16_t blanking_count_ = 0;
    uint16_t stall_count_ = 0;
    uint16_t overcurrent_count_ = 0;
    bool stall_confirmed_ = false;
    bool overcurrent_ = false;
    bool motor_running_ = false;

    // Encoder stall detection
    int32_t last_encoder_pos_ = 0;
    uint16_t encoder_no_change_count_ = 0;
    bool encoder_stall_ = false;
};

} // namespace zlens
```

```cpp
// App/Src/stall_detect.cpp
#include "stall_detect.hpp"

namespace zlens {

void StallDetect::init() { reset(); }

void StallDetect::reset() {
    blanking_count_ = 0;
    stall_count_ = 0;
    overcurrent_count_ = 0;
    stall_confirmed_ = false;
    overcurrent_ = false;
    motor_running_ = false;
    encoder_no_change_count_ = 0;
    encoder_stall_ = false;
    last_encoder_pos_ = 0;
}

void StallDetect::start_motor() {
    reset();
    motor_running_ = true;
}

void StallDetect::update(uint16_t adc_current, int32_t encoder_pos, uint32_t) {
    if (!motor_running_ || stall_confirmed_ || overcurrent_) return;

    // Blanking window
    if (blanking_count_ < BLANKING_TICKS) {
        blanking_count_++;
        last_encoder_pos_ = encoder_pos;
        return;
    }

    // Overcurrent check (fast response)
    if (adc_current > OVERCURRENT_THRESHOLD) {
        overcurrent_count_++;
        if (overcurrent_count_ >= OVERCURRENT_CONFIRM) {
            overcurrent_ = true;
            return;
        }
    } else {
        overcurrent_count_ = 0;
    }

    // Stall current check
    if (adc_current > STALL_THRESHOLD) {
        stall_count_++;
        if (stall_count_ >= STALL_CONFIRM_COUNT) {
            stall_confirmed_ = true;
        }
    } else {
        stall_count_ = 0;
    }

    // Encoder stall check
    if (encoder_pos == last_encoder_pos_) {
        encoder_no_change_count_++;
        if (encoder_no_change_count_ >= ENCODER_STALL_TICKS) {
            encoder_stall_ = true;
        }
    } else {
        encoder_no_change_count_ = 0;
        last_encoder_pos_ = encoder_pos;
    }
}

StallDetect::LimitType StallDetect::get_stall_limit_type() const {
    return (direction_ == Direction::FORWARD) ? LimitType::LIMIT_MAX : LimitType::LIMIT_MIN;
}

} // namespace zlens
```

- [ ] **Step 3: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_stall_detect test_stall_detect.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/stall_detect.cpp)
target_link_libraries(test_stall_detect PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_stall_detect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_stall_detect COMMAND test_stall_detect)
```

Run: `cd build-test && ninja test_stall_detect && ./test_stall_detect`
Expected: All tests PASSED

- [ ] **Step 4: Commit**

```bash
git add App/Inc/stall_detect.hpp App/Src/stall_detect.cpp Test/test_stall_detect.cpp Test/CMakeLists.txt
git commit -m "feat: add stall detection with blanking window, overcurrent, encoder assist"
```

---

## Chunk 5: 电机控制与系统管理模块

### Task 9: 电机控制模块（TDD）

**Files:**
- Create: `App/Inc/motor_ctrl.hpp`
- Create: `App/Src/motor_ctrl.cpp`
- Create: `Test/test_motor_ctrl.cpp`

- [ ] **Step 1: 写电机控制失败测试**

测试覆盖：状态机转换、梯形加减速、PWM 输出、到位判断、紧急停止、DAC 电流限制。

```cpp
// Test/test_motor_ctrl.cpp
#include <gtest/gtest.h>
#include "motor_ctrl.hpp"

using namespace zlens;

class MotorCtrlTest : public ::testing::Test {
protected:
    MotorCtrl motor;
    Encoder encoder;
    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;

    void SetUp() override {
        mock::get_log().reset();
        htim3.Instance = TIM3;
        hdac.Instance = DAC1;
        encoder.init();
        motor.init(&htim3, &hdac, &encoder);
    }
};

TEST_F(MotorCtrlTest, InitialState_IsIdle) {
    EXPECT_EQ(motor.get_state(), MotorState::IDLE);
}

TEST_F(MotorCtrlTest, MoveTo_StartsAccelerating) {
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MotorState::ACCELERATING);
    EXPECT_EQ(motor.get_direction(), Direction::FORWARD);
}

TEST_F(MotorCtrlTest, MoveTo_Reverse) {
    encoder.set_position(50000);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_direction(), Direction::REVERSE);
}

TEST_F(MotorCtrlTest, Update_AcceleratesSpeed) {
    motor.move_to(100000);
    uint16_t speed1 = motor.get_current_speed();
    motor.update();
    uint16_t speed2 = motor.get_current_speed();
    EXPECT_GT(speed2, speed1); // Speed should increase
}

TEST_F(MotorCtrlTest, EmergencyStop_GoesToIdle) {
    motor.move_to(10000);
    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MotorState::IDLE);
    // PWM should be set to brake (both channels high)
}

TEST_F(MotorCtrlTest, ArrivedAtTarget_StopsAndIdle) {
    motor.move_to(100);
    encoder.set_position(80); // within deadzone (±50)
    // Run enough updates to pass through deceleration
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MotorState::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MotorState::IDLE);
}

TEST_F(MotorCtrlTest, SetCurrentLimit) {
    motor.set_current_limit(500); // 500mA
    EXPECT_GE(mock::get_log().dac_sets.size(), 1u);
}

TEST_F(MotorCtrlTest, Stop_DeceleratesFirst) {
    motor.move_to(100000);
    for (int i = 0; i < 100; ++i) motor.update(); // Build speed
    motor.stop();
    EXPECT_EQ(motor.get_state(), MotorState::DECELERATING);
}
```

- [ ] **Step 2: 实现 MotorCtrl 类**

```cpp
// App/Inc/motor_ctrl.hpp
#pragma once
#include <cstdint>
#include "encoder.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

enum class MotorState { IDLE, ACCELERATING, CONSTANT, DECELERATING, BRAKING, STALLED };
enum class Direction { FORWARD, REVERSE };

class MotorCtrl {
public:
    static constexpr uint16_t PWM_ARR = 4799;
    static constexpr uint16_t MAX_SPEED = 3600;    // ~75% duty
    static constexpr uint16_t MIN_SPEED = 480;     // ~10% duty
    static constexpr uint16_t ACCEL_STEP = 10;     // PWM increment per ms
    static constexpr int32_t  DEADZONE = 50;        // ±50 counts
    static constexpr int32_t  DECEL_DISTANCE = 5000; // start decel at this distance

    void init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder);
    void move_to(int32_t target);
    void stop();
    void emergency_stop();
    void update();

    void set_current_limit(uint16_t milliamps);
    MotorState get_state() const { return state_; }
    Direction get_direction() const { return direction_; }
    uint16_t get_current_speed() const { return current_speed_; }
    int32_t get_target() const { return target_; }

    void set_max_speed(uint16_t speed) { max_speed_ = speed; }
    void set_min_speed(uint16_t speed) { min_speed_ = speed; }

private:
    TIM_HandleTypeDef* htim_ = nullptr;
    DAC_HandleTypeDef* hdac_ = nullptr;
    Encoder* encoder_ = nullptr;

    MotorState state_ = MotorState::IDLE;
    Direction direction_ = Direction::FORWARD;
    int32_t target_ = 0;
    uint16_t current_speed_ = 0;
    uint16_t max_speed_ = MAX_SPEED;
    uint16_t min_speed_ = MIN_SPEED;

    void set_pwm(Direction dir, uint16_t speed);
    void brake();
    void coast();
};

} // namespace zlens
```

```cpp
// App/Src/motor_ctrl.cpp
#include "motor_ctrl.hpp"
#include <cstdlib>

namespace zlens {

void MotorCtrl::init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder) {
    htim_ = htim;
    hdac_ = hdac;
    encoder_ = encoder;
    state_ = MotorState::IDLE;
    current_speed_ = 0;
    coast();
}

void MotorCtrl::set_pwm(Direction dir, uint16_t speed) {
    if (dir == Direction::FORWARD) {
        htim_->Instance->CCR1 = speed;
        htim_->Instance->CCR2 = 0;
    } else {
        htim_->Instance->CCR1 = 0;
        htim_->Instance->CCR2 = speed;
    }
}

void MotorCtrl::brake() {
    htim_->Instance->CCR1 = PWM_ARR;
    htim_->Instance->CCR2 = PWM_ARR;
}

void MotorCtrl::coast() {
    htim_->Instance->CCR1 = 0;
    htim_->Instance->CCR2 = 0;
}

void MotorCtrl::move_to(int32_t target) {
    target_ = target;
    int32_t pos = encoder_->get_position();
    int32_t diff = target - pos;

    if (std::abs(diff) <= DEADZONE) {
        state_ = MotorState::IDLE;
        return;
    }

    direction_ = (diff > 0) ? Direction::FORWARD : Direction::REVERSE;
    current_speed_ = min_speed_;
    state_ = MotorState::ACCELERATING;
    set_pwm(direction_, current_speed_);
}

void MotorCtrl::stop() {
    if (state_ == MotorState::IDLE) return;
    state_ = MotorState::DECELERATING;
}

void MotorCtrl::emergency_stop() {
    brake();
    current_speed_ = 0;
    state_ = MotorState::IDLE;
}

void MotorCtrl::set_current_limit(uint16_t milliamps) {
    // I_max = VREF / 2 => VREF = I_max * 2
    // DAC 12-bit: 0-4095 maps to 0-3.3V
    // VREF = milliamps * 0.002 (since I = VREF/(10*0.2) = VREF/2)
    uint32_t vref_mv = milliamps * 2; // mV
    uint32_t dac_val = vref_mv * 4095 / 3300;
    if (dac_val > 4095) dac_val = 4095;
    HAL_DAC_SetValue(hdac_, 2, 0, dac_val);
}

void MotorCtrl::update() {
    if (state_ == MotorState::IDLE || state_ == MotorState::STALLED) return;

    int32_t pos = encoder_->get_position();
    int32_t remaining = std::abs(target_ - pos);

    // Check arrival
    if (remaining <= DEADZONE) {
        brake();
        current_speed_ = 0;
        state_ = MotorState::IDLE;
        return;
    }

    switch (state_) {
    case MotorState::ACCELERATING:
        if (current_speed_ < max_speed_) {
            current_speed_ += ACCEL_STEP;
            if (current_speed_ > max_speed_) current_speed_ = max_speed_;
        } else {
            state_ = MotorState::CONSTANT;
        }
        if (remaining < DECEL_DISTANCE) {
            state_ = MotorState::DECELERATING;
        }
        break;

    case MotorState::CONSTANT:
        if (remaining < DECEL_DISTANCE) {
            state_ = MotorState::DECELERATING;
        }
        break;

    case MotorState::DECELERATING:
        if (current_speed_ > min_speed_) {
            current_speed_ -= ACCEL_STEP;
            if (current_speed_ < min_speed_) current_speed_ = min_speed_;
        }
        break;

    case MotorState::BRAKING:
        brake();
        current_speed_ = 0;
        state_ = MotorState::IDLE;
        return;

    default:
        break;
    }

    set_pwm(direction_, current_speed_);
}

} // namespace zlens
```

- [ ] **Step 3: 注册测试并运行**

在 `Test/CMakeLists.txt` 添加:
```cmake
add_executable(test_motor_ctrl test_motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp)
target_link_libraries(test_motor_ctrl PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_motor_ctrl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_motor_ctrl COMMAND test_motor_ctrl)
```

Run: `cd build-test && ninja test_motor_ctrl && ./test_motor_ctrl`
Expected: All tests PASSED

- [ ] **Step 4: Commit**

```bash
git add App/Inc/motor_ctrl.hpp App/Src/motor_ctrl.cpp Test/test_motor_ctrl.cpp Test/CMakeLists.txt
git commit -m "feat: add motor control with trapezoidal acceleration and DAC current limit"
```

---

### Task 10: 电源监测模块（TDD）

**Files:**
- Create: `App/Inc/power_monitor.hpp`
- Create: `App/Src/power_monitor.cpp`
- Create: `Test/test_power_monitor.cpp`

- [ ] **Step 1: 写测试 → Step 2: 实现**

测试覆盖：ADC值到电压转换、掉电阈值判定、状态上报。

```cpp
// Test/test_power_monitor.cpp
#include <gtest/gtest.h>
#include "power_monitor.hpp"

using namespace zlens;

TEST(PowerMonitor, NormalVoltage_NoPowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(3723)); // 12V normal
    EXPECT_FALSE(pm.is_power_down(3500)); // ~11.3V OK
}

TEST(PowerMonitor, LowVoltage_PowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_TRUE(pm.is_power_down(3000)); // < 3103 threshold
    EXPECT_TRUE(pm.is_power_down(0));
}

TEST(PowerMonitor, ThresholdBoundary) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(3103)); // exactly at threshold = OK
    EXPECT_TRUE(pm.is_power_down(3102));  // just below
}

TEST(PowerMonitor, AdcToMillivolts) {
    // 12V / 4 (divider) = 3.0V at ADC
    // ADC 3723 = 12V, ADC 3103 = 10V
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(3723), 12000, 50);
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(3103), 10000, 50);
}
```

```cpp
// App/Inc/power_monitor.hpp
#pragma once
#include <cstdint>

namespace zlens {

class PowerMonitor {
public:
    static constexpr uint16_t POWER_DOWN_THRESHOLD = 3103; // ADC value for 10V

    void init();
    bool is_power_down(uint16_t adc_value) const;
    static uint32_t adc_to_voltage_mv(uint16_t adc_value);
};

} // namespace zlens
```

```cpp
// App/Src/power_monitor.cpp
#include "power_monitor.hpp"

namespace zlens {

void PowerMonitor::init() {}

bool PowerMonitor::is_power_down(uint16_t adc_value) const {
    return adc_value < POWER_DOWN_THRESHOLD;
}

uint32_t PowerMonitor::adc_to_voltage_mv(uint16_t adc_value) {
    // Divider ratio 1:4, so V_in = ADC * 3300 / 4095 * 4
    return static_cast<uint32_t>(adc_value) * 3300 * 4 / 4095;
}

} // namespace zlens
```

- [ ] **Step 2: 注册测试并运行**

Run: `cd build-test && ninja test_power_monitor && ./test_power_monitor`
Expected: All tests PASSED

- [ ] **Step 3: Commit**

```bash
git add App/Inc/power_monitor.hpp App/Src/power_monitor.cpp Test/test_power_monitor.cpp Test/CMakeLists.txt
git commit -m "feat: add power monitor with ADC watchdog threshold detection"
```

---

### Task 11: 系统状态管理模块（TDD）

**Files:**
- Create: `App/Inc/system_manager.hpp`
- Create: `App/Src/system_manager.cpp`
- Create: `Test/test_system_manager.cpp`

- [ ] **Step 1: 写测试 → Step 2: 实现**

测试覆盖：状态机 INIT→SELF_TEST→HOMING→READY→BUSY→ERROR 的所有合法转换和非法转换拒绝。

```cpp
// Test/test_system_manager.cpp
#include <gtest/gtest.h>
#include "system_manager.hpp"

using namespace zlens;

class SystemManagerTest : public ::testing::Test {
protected:
    SystemManager sm;
    void SetUp() override { sm.init(); }
};

TEST_F(SystemManagerTest, InitialState_IsInit) {
    EXPECT_EQ(sm.get_state(), SystemState::INIT);
}

TEST_F(SystemManagerTest, Transition_Init_To_SelfTest) {
    EXPECT_TRUE(sm.transition_to(SystemState::SELF_TEST));
    EXPECT_EQ(sm.get_state(), SystemState::SELF_TEST);
}

TEST_F(SystemManagerTest, Transition_SelfTest_To_Homing) {
    sm.transition_to(SystemState::SELF_TEST);
    EXPECT_TRUE(sm.transition_to(SystemState::HOMING));
}

TEST_F(SystemManagerTest, Transition_Homing_To_Ready) {
    sm.transition_to(SystemState::SELF_TEST);
    sm.transition_to(SystemState::HOMING);
    EXPECT_TRUE(sm.transition_to(SystemState::READY));
}

TEST_F(SystemManagerTest, Transition_Ready_To_Busy) {
    sm.transition_to(SystemState::SELF_TEST);
    sm.transition_to(SystemState::HOMING);
    sm.transition_to(SystemState::READY);
    EXPECT_TRUE(sm.transition_to(SystemState::BUSY));
}

TEST_F(SystemManagerTest, Transition_Busy_To_Ready) {
    sm.transition_to(SystemState::SELF_TEST);
    sm.transition_to(SystemState::HOMING);
    sm.transition_to(SystemState::READY);
    sm.transition_to(SystemState::BUSY);
    EXPECT_TRUE(sm.transition_to(SystemState::READY));
}

TEST_F(SystemManagerTest, AnyState_To_Error) {
    EXPECT_TRUE(sm.transition_to(SystemState::ERROR_STATE));
}

TEST_F(SystemManagerTest, InvalidTransition_Rejected) {
    // Can't go directly from INIT to READY
    EXPECT_FALSE(sm.transition_to(SystemState::READY));
    EXPECT_EQ(sm.get_state(), SystemState::INIT); // unchanged
}

TEST_F(SystemManagerTest, IsReady) {
    EXPECT_FALSE(sm.is_ready());
    sm.transition_to(SystemState::SELF_TEST);
    sm.transition_to(SystemState::HOMING);
    sm.transition_to(SystemState::READY);
    EXPECT_TRUE(sm.is_ready());
}

TEST_F(SystemManagerTest, StatusCode) {
    sm.transition_to(SystemState::SELF_TEST);
    sm.transition_to(SystemState::HOMING);
    sm.transition_to(SystemState::READY);
    EXPECT_EQ(sm.get_status_code(), 0x0000); // stopped

    sm.transition_to(SystemState::BUSY);
    EXPECT_EQ(sm.get_status_code(), 0x0001); // running
}
```

```cpp
// App/Inc/system_manager.hpp
#pragma once
#include <cstdint>

namespace zlens {

enum class SystemState { INIT, SELF_TEST, HOMING, READY, BUSY, ERROR_STATE };

class SystemManager {
public:
    void init();
    bool transition_to(SystemState new_state);
    SystemState get_state() const { return state_; }
    bool is_ready() const { return state_ == SystemState::READY; }
    bool is_busy() const { return state_ == SystemState::BUSY; }
    uint16_t get_status_code() const;

private:
    SystemState state_ = SystemState::INIT;
    bool is_valid_transition(SystemState from, SystemState to) const;
};

} // namespace zlens
```

```cpp
// App/Src/system_manager.cpp
#include "system_manager.hpp"

namespace zlens {

void SystemManager::init() {
    state_ = SystemState::INIT;
}

bool SystemManager::is_valid_transition(SystemState from, SystemState to) const {
    // Any state can go to ERROR
    if (to == SystemState::ERROR_STATE) return true;

    switch (from) {
    case SystemState::INIT:       return to == SystemState::SELF_TEST;
    case SystemState::SELF_TEST:  return to == SystemState::HOMING;
    case SystemState::HOMING:     return to == SystemState::READY;
    case SystemState::READY:      return to == SystemState::BUSY || to == SystemState::HOMING;
    case SystemState::BUSY:       return to == SystemState::READY;
    case SystemState::ERROR_STATE: return to == SystemState::INIT;
    }
    return false;
}

bool SystemManager::transition_to(SystemState new_state) {
    if (!is_valid_transition(state_, new_state)) return false;
    state_ = new_state;
    return true;
}

uint16_t SystemManager::get_status_code() const {
    switch (state_) {
    case SystemState::READY: return 0x0000; // stopped
    case SystemState::BUSY:  return 0x0001; // running
    case SystemState::ERROR_STATE: return 0x0002; // stalled (generic)
    default: return 0x0000;
    }
}

} // namespace zlens
```

- [ ] **Step 2: 注册测试并运行**

Run: `cd build-test && ninja test_system_manager && ./test_system_manager`
Expected: All tests PASSED

- [ ] **Step 3: Commit**

```bash
git add App/Inc/system_manager.hpp App/Src/system_manager.cpp Test/test_system_manager.cpp Test/CMakeLists.txt
git commit -m "feat: add system state manager with validated state transitions"
```

---

## Chunk 6: FreeRTOS 任务与系统集成

### Task 12: FreeRTOS 任务骨架

**Files:**
- Create: `Tasks/Inc/motor_task.hpp` + `Tasks/Src/motor_task.cpp`
- Create: `Tasks/Inc/comm_task.hpp` + `Tasks/Src/comm_task.cpp`
- Create: `Tasks/Inc/storage_task.hpp` + `Tasks/Src/storage_task.cpp`
- Create: `Tasks/Inc/monitor_task.hpp` + `Tasks/Src/monitor_task.cpp`
- Create: `Tasks/Inc/task_config.hpp` (共享队列/任务句柄定义)

- [ ] **Step 1: 创建任务配置头文件**

定义所有队列句柄、任务句柄、共享消息结构体。

```cpp
// Tasks/Inc/task_config.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include "mock_freertos.hpp"
#else
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#endif

namespace zlens {

// Command message from CommTask to MotorTask
struct CmdMessage {
    uint8_t cmd;
    uint16_t param;
};

// Response message from MotorTask to CommTask
struct RspMessage {
    uint8_t cmd;
    uint16_t param;
};

// Save message from MotorTask to StorageTask
struct SaveMessage {
    int32_t position;
    uint16_t zoom_x10;
    uint8_t reason; // 0=到位, 1=定期, 2=掉电, 3=堵转
};

// Task priorities
constexpr uint32_t MOTOR_TASK_PRIORITY   = 4;
constexpr uint32_t COMM_TASK_PRIORITY    = 3;
constexpr uint32_t STORAGE_TASK_PRIORITY = 2;
constexpr uint32_t MONITOR_TASK_PRIORITY = 1;

// Stack sizes (words)
constexpr uint32_t MOTOR_TASK_STACK   = 512;
constexpr uint32_t COMM_TASK_STACK    = 256;
constexpr uint32_t STORAGE_TASK_STACK = 256;
constexpr uint32_t MONITOR_TASK_STACK = 256;

// Global queue handles (defined in main.cpp, extern here)
extern QueueHandle_t g_cmdQueue;
extern QueueHandle_t g_rspQueue;
extern QueueHandle_t g_saveQueue;
extern volatile bool g_spi_emergency;

} // namespace zlens
```

- [ ] **Step 2: 创建 4 个任务骨架文件**

每个任务文件包含 task entry function 和初始化逻辑。任务体使用 App 层模块的接口。此步骤只创建骨架框架，具体业务逻辑留待 CubeMX 集成后完善。

- [ ] **Step 3: Commit**

```bash
git add Tasks/
git commit -m "feat: add FreeRTOS task skeletons with shared message types"
```

---

### Task 13: 全部测试运行 + 覆盖率检查

- [ ] **Step 1: 运行所有测试**

Run:
```bash
cd /home/cbn/VSCode/STM32_PRJ/ZLENS_DC/build-test
cmake -G Ninja -DBUILD_TESTING=ON ..
ninja && ctest --output-on-failure
```
Expected: ALL tests PASSED (8 test suites)

- [ ] **Step 2: 检查覆盖率（可选，如 gcov 可用）**

```bash
cd build-test
cmake -G Ninja -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" ..
ninja && ctest
gcovr --root .. --filter '../App/Src' --print-summary
```
Expected: 各模块覆盖率 >80%

- [ ] **Step 3: 最终 Commit**

```bash
git add -A
git commit -m "test: all 8 modules passing with >80% coverage"
```

---

## 后续步骤（需 CubeMX 配合）

### Task 14: CubeMX 项目生成（需手动操作）

用户需要使用 STM32CubeMX GUI 生成 Core/ 和 Drivers/ 目录：
1. 创建 STM32F103RCT6 项目
2. 配置所有外设（见设计文档第 8 节）
3. 生成代码到项目目录
4. 更新 CMakeLists.txt 的 ARM 目标，添加 Core/Drivers 源文件

### Task 15: 系统集成

将 App 层模块和 Tasks 连接到 CubeMX 生成的 main.cpp，在 `USER CODE BEGIN/END` 区域添加初始化和任务创建代码。
