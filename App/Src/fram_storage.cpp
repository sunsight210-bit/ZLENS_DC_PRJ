// App/Src/fram_storage.cpp
#include "fram_storage.hpp"
#include "crc16.hpp"
#include <cstddef>
#include <cstring>

namespace zlens {

void FramStorage::init(SPI_HandleTypeDef* hspi) {
    m_pHspi = hspi;
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
    HAL_SPI_Transmit(m_pHspi, &cmd, 1, 100);
    cs_high();
}

void FramStorage::spi_write(uint16_t addr, const uint8_t* data, uint16_t len) {
    write_disable_wp();
    write_enable();
    cs_low();
    uint8_t hdr[3] = {0x02, static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF)};
    HAL_SPI_Transmit(m_pHspi, hdr, 3, 100);
    HAL_SPI_Transmit(m_pHspi, const_cast<uint8_t*>(data), len, 100);
    cs_high();
    enable_wp();
}

void FramStorage::spi_read(uint16_t addr, uint8_t* data, uint16_t len) {
    cs_low();
    uint8_t hdr[3] = {0x03, static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF)};
    HAL_SPI_Transmit(m_pHspi, hdr, 3, 100);
    HAL_SPI_Receive(m_pHspi, data, len, 100);
    cs_high();
}

uint16_t FramStorage::calc_crc(const FRAM_PARAMS_S& p) {
    // CRC over all bytes except the last 2 (crc16 field itself)
    return crc16_modbus(reinterpret_cast<const uint8_t*>(&p), sizeof(FRAM_PARAMS_S) - 2);
}

bool FramStorage::verify_crc(const FRAM_PARAMS_S& p) {
    return calc_crc(p) == p.crc16;
}

bool FramStorage::check_magic(const FRAM_PARAMS_S& p) {
    return p.magic_number == MAGIC;
}

bool FramStorage::save_params(const FRAM_PARAMS_S& params) {
    FRAM_PARAMS_S p = params;
    p.crc16 = calc_crc(p);
    spi_write(PRIMARY_ADDR, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    spi_write(BACKUP_ADDR, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    return true;
}

bool FramStorage::load_params(FRAM_PARAMS_S& params) {
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

    uint16_t pos_addr = PRIMARY_ADDR + offsetof(FRAM_PARAMS_S, current_position);
    uint8_t hdr[3] = {0x02, static_cast<uint8_t>(pos_addr >> 8), static_cast<uint8_t>(pos_addr & 0xFF)};
    HAL_SPI_Transmit(m_pHspi, hdr, 3, 100);

    // Write position (4 bytes)
    HAL_SPI_Transmit(m_pHspi, reinterpret_cast<uint8_t*>(&position), 4, 100);
    cs_high();

    // Write position_valid = 0xFF
    uint16_t valid_addr = PRIMARY_ADDR + offsetof(FRAM_PARAMS_S, position_valid);
    write_enable();
    cs_low();
    uint8_t hdr2[3] = {0x02, static_cast<uint8_t>(valid_addr >> 8), static_cast<uint8_t>(valid_addr & 0xFF)};
    HAL_SPI_Transmit(m_pHspi, hdr2, 3, 100);
    uint8_t valid_data[1] = {0xFF};
    HAL_SPI_Transmit(m_pHspi, valid_data, 1, 100);
    cs_high();

    enable_wp();
}

bool FramStorage::is_valid() {
    FRAM_PARAMS_S p{};
    return load_params(p);
}

void FramStorage::load_params_from_buffer(const FRAM_PARAMS_S& src, FRAM_PARAMS_S& dst) {
    dst = src;
}

} // namespace zlens
