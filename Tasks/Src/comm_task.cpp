// Tasks/Src/comm_task.cpp
#include "comm_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#endif

namespace zlens {

void CommTask::init(CommProtocol* pComm, SystemManager* pSm, ZoomTable* pZoom,
                    QueueHandle_t cmdQ, QueueHandle_t rspQ,
                    UART_HandleTypeDef* pHuart) {
    m_pComm = pComm;
    m_pSm = pSm;
    m_pZoom = pZoom;
    m_cmdQueue = cmdQ;
    m_rspQueue = rspQ;
    m_pHuart = pHuart;
    m_iCurrentZoom = 60; // default 6.0x
}

void CommTask::run_once() {
    // Check response queue and forward to UART
    RSP_MESSAGE_S stRsp;
    if (xQueueReceive(m_rspQueue, &stRsp, 0) == pdTRUE) {
        // Update current zoom if it's a zoom response
        if (stRsp.cmd == cmd::SET_ZOOM) {
            m_iCurrentZoom = stRsp.param;
        }
        send_uart_frame(stRsp.cmd, stRsp.param);
    }
}

void CommTask::on_frame_received(const uint8_t* data, uint16_t len) {
    if (m_pComm->is_factory_mode()) {
        auto result = m_pComm->parse_factory_frame(data, len);
        if (result.valid) {
            dispatch_factory_command(result.cmd, result.param_high, result.param_low);
        }
    } else {
        auto result = m_pComm->parse_work_frame(data, len);
        if (result.valid) {
            dispatch_work_command(result.cmd, result.param);
        }
    }
}

void CommTask::dispatch_work_command(uint8_t cmd_byte, uint16_t param) {
    // Handle queries directly (no forwarding to MotorTask)
    if (cmd_byte >= cmd::QUERY_ZOOM && cmd_byte <= cmd::QUERY_TYPE) {
        handle_query(cmd_byte);
        return;
    }

    // Handshake: echo back
    if (cmd_byte == cmd::HANDSHAKE) {
        send_uart_frame(cmd::HANDSHAKE, param);
        return;
    }

    // Switch factory mode
    if (cmd_byte == cmd::SWITCH_FACTORY) {
        m_pComm->set_factory_mode(!m_pComm->is_factory_mode());
        send_uart_frame(cmd::SWITCH_FACTORY,
                        m_pComm->is_factory_mode() ? 0x0001 : 0x0000);
        return;
    }

    // Self-test: set flag + ACK
    if (cmd_byte == cmd::SELF_TEST) {
        g_bUartSelfTestReq = true;
        send_uart_frame(cmd::SELF_TEST, rsp::OK);
        return;
    }

    // Motion commands: reject if busy
    if (is_motion_command(cmd_byte) && m_pSm->is_busy()) {
        send_uart_frame(cmd_byte, rsp::BUSY);
        return;
    }

    // Forward to MotorTask via command queue
    CMD_MESSAGE_S stCmd = {cmd_byte, param};
    xQueueSend(m_cmdQueue, &stCmd, 0);
}

void CommTask::dispatch_factory_command(uint8_t cmd_byte, uint16_t paramH, uint16_t paramL) {
    switch (cmd_byte) {
    case fcmd::SET_ANGLE: {
        // Forward as CMD to MotorTask (param = angle_x100)
        CMD_MESSAGE_S stCmd = {cmd_byte, paramL};
        xQueueSend(m_cmdQueue, &stCmd, 0);
        break;
    }
    default:
        break;
    }
}

void CommTask::handle_query(uint8_t cmd_byte) {
    switch (cmd_byte) {
    case cmd::QUERY_ZOOM:
        send_uart_frame(cmd::QUERY_ZOOM, m_iCurrentZoom);
        break;
    case cmd::QUERY_STATUS:
        send_uart_frame(cmd::QUERY_STATUS, m_pSm->get_status_code());
        break;
    case cmd::QUERY_TYPE:
        send_uart_frame(cmd::QUERY_TYPE, rsp::LENS_TYPE);
        break;
    case cmd::QUERY_RANGE:
        // Send two frames: min zoom then max zoom
        send_uart_frame(cmd::QUERY_RANGE, m_pZoom->get_min_zoom());
        send_uart_frame(cmd::QUERY_RANGE, m_pZoom->get_max_zoom());
        break;
    default:
        break;
    }
}

void CommTask::send_uart_frame(uint8_t cmd_byte, uint16_t param) {
    uint8_t aFrame[CommProtocol::WORK_FRAME_SIZE];
    m_pComm->build_work_frame(cmd_byte, param, aFrame);
    HAL_UART_Transmit(m_pHuart, aFrame, CommProtocol::WORK_FRAME_SIZE, 100);
}

bool CommTask::is_motion_command(uint8_t cmd_byte) const {
    return cmd_byte == cmd::SET_ZOOM ||
           cmd_byte == cmd::HOMING ||
           cmd_byte == cmd::CYCLE_START;
}

} // namespace zlens

#ifndef BUILD_TESTING
#include "swo_debug.hpp"

// UART DMA receive buffer (file-scope, used by ISR callback)
static uint8_t s_aRxBuf[16];
static volatile uint16_t s_iRxLen = 0;
static volatile bool s_bRxReady = false;

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        s_iRxLen = Size;
        s_bRxReady = true;
    }
}

static void start_uart_dma_rx(UART_HandleTypeDef* huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_aRxBuf, sizeof(s_aRxBuf));
    // Disable DMA half-transfer interrupt (common ReceiveToIdle_DMA pitfall)
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
#endif

extern "C" void comm_task_entry(void* params) {
#ifndef BUILD_TESTING
    (void)params;
    using namespace zlens;

    extern UART_HandleTypeDef huart2;

    static CommTask task;
    task.init(&g_CommProtocol, &g_SystemManager, &g_ZoomTable,
              g_cmdQueue, g_rspQueue, &huart2);

    // Start UART DMA receive (IDLE line detection)
    start_uart_dma_rx(&huart2);
    swo_printf("[COMM] Task started, UART DMA RX armed\n");

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        if (s_bRxReady) {
            s_bRxReady = false;
            swo_printf("[COMM] RX %u bytes:", s_iRxLen);
            for (uint16_t i = 0; i < s_iRxLen && i < 8; i++) {
                swo_printf(" %02X", s_aRxBuf[i]);
            }
            swo_printf("\n");
            task.on_frame_received(s_aRxBuf, s_iRxLen);
            start_uart_dma_rx(&huart2);
        }
        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }
#else
    (void)params;
#endif
}
