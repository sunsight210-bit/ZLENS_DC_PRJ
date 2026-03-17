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
