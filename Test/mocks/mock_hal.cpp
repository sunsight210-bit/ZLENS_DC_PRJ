// Test/mocks/mock_hal.cpp
#include "mock_hal.hpp"
#include <algorithm>

// Peripheral instances
TIM_TypeDef mock_TIM3_instance = {};
TIM_TypeDef mock_TIM4_instance = {};
TIM_TypeDef mock_TIM8_instance = {};
SPI_TypeDef mock_SPI2_instance = {};
GPIO_TypeDef mock_GPIOB_instance = {};
GPIO_TypeDef mock_GPIOA_instance = {};
ADC_TypeDef mock_ADC1_instance = {};
DAC_TypeDef mock_DAC_instance = {};
IWDG_TypeDef mock_IWDG_instance = {};

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
    iwdg_refresh_count = 0;
    tick = 0;
    tick_auto_increment = true;
    mock_TIM3_instance = {};
    mock_TIM4_instance = {};
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
    (void)port; (void)pin;
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
    (void)pEraseInit;
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

uint32_t HAL_GetTick(void) {
    auto& log = mock::get_log();
    if (log.tick_auto_increment) {
        return log.tick++;
    }
    return log.tick;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef*) {
    mock::get_log().iwdg_refresh_count++;
    return HAL_OK;
}

} // extern "C"
