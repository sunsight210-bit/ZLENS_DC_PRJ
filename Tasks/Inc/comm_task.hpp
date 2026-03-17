// Tasks/Inc/comm_task.hpp
#pragma once
#include "task_config.hpp"
#include "comm_protocol.hpp"
#include "system_manager.hpp"
#include "zoom_table.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

class CommTask {
public:
    void init(CommProtocol* pComm, SystemManager* pSm, ZoomTable* pZoom,
              QueueHandle_t cmdQ, QueueHandle_t rspQ,
              UART_HandleTypeDef* pHuart);

    void run_once();
    void on_frame_received(const uint8_t* data, uint16_t len);

    uint16_t get_current_zoom() const { return m_iCurrentZoom; }
    void set_current_zoom(uint16_t zoom) { m_iCurrentZoom = zoom; }

private:
    CommProtocol* m_pComm = nullptr;
    SystemManager* m_pSm = nullptr;
    ZoomTable* m_pZoom = nullptr;
    UART_HandleTypeDef* m_pHuart = nullptr;

    QueueHandle_t m_cmdQueue = nullptr;
    QueueHandle_t m_rspQueue = nullptr;

    uint16_t m_iCurrentZoom = 60; // default 6.0x (0.6x = 6, 60x = 600)

    void dispatch_work_command(uint8_t cmd, uint16_t param);
    void dispatch_factory_command(uint8_t cmd, uint16_t paramH, uint16_t paramL);
    void send_uart_frame(uint8_t cmd, uint16_t param);
    void handle_query(uint8_t cmd);
    bool is_motion_command(uint8_t cmd) const;
};

} // namespace zlens

extern "C" void comm_task_entry(void* params);
