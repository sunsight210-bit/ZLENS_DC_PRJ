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

    // IWDG
    uint32_t iwdg_refresh_count = 0;

    // Controllable tick
    uint32_t tick = 0;
    bool tick_auto_increment = true;

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

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef* hiwdg);

#ifdef __cplusplus
}
#endif
