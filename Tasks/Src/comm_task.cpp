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
    m_iSpeedKhz = rsp::DEFAULT_SPEED_KHZ;
    m_iStallCount = 0;
}

void CommTask::run_once() {
    // Check response queue and forward to UART
    RSP_MESSAGE_S stRsp;
    if (xQueueReceive(m_rspQueue, &stRsp, 0) == pdTRUE) {
        // Update current zoom if it's a zoom response
        if (stRsp.cmd == rsp_cmd::ZOOM) {
            m_iCurrentZoom = stRsp.param;
        }
        send_uart_frame(stRsp.cmd, stRsp.param);
    }
}

void CommTask::on_frame_received(const uint8_t* data, uint16_t len) {
    if (m_pComm->is_factory_mode()) {
        auto result = m_pComm->parse_factory_frame(data, len);
        if (result.valid) {
            // Handshake: echo raw frame
            HAL_UART_Transmit(m_pHuart, const_cast<uint8_t*>(data), len, 100);
            dispatch_factory_command(result.cmd, result.param_high, result.param_low);
        }
    } else {
        auto result = m_pComm->parse_work_frame(data, len);
        if (result.valid) {
            // Handshake: echo raw frame
            HAL_UART_Transmit(m_pHuart, const_cast<uint8_t*>(data), len, 100);
            dispatch_work_command(result.cmd, result.param);
        }
    }
}

void CommTask::dispatch_work_command(uint8_t cmd_byte, uint16_t param) {
    // Query commands: direct response
    if (cmd_byte >= cmd::QUERY_ZOOM && cmd_byte <= cmd::QUERY_VERSION) {
        handle_query(cmd_byte);
        return;
    }

    // GET_STALL_CNT: direct response
    if (cmd_byte == cmd::GET_STALL_CNT) {
        send_uart_frame(rsp_cmd::STALL_COUNT, m_iStallCount);
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
        send_uart_frame(rsp_cmd::REQ_INVALID, rsp::REQ_INVALID_PARAM);
        return;
    }

    // Forward to MotorTask via command queue
    CMD_MESSAGE_S stCmd = {cmd_byte, param};
    xQueueSend(m_cmdQueue, &stCmd, 0);
}

void CommTask::dispatch_factory_command(uint8_t cmd_byte, uint16_t paramH, uint16_t paramL) {
    switch (cmd_byte) {
    case fcmd::SET_ANGLE: {
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
        send_uart_frame(rsp_cmd::ZOOM, m_iCurrentZoom);
        break;
    case cmd::QUERY_STATUS:
        send_uart_frame(rsp_cmd::STATUS, m_pSm->get_status_code());
        break;
    case cmd::QUERY_SPEED:
        send_uart_frame(rsp_cmd::SPEED, m_iSpeedKhz);
        break;
    case cmd::QUERY_TYPE:
        send_uart_frame(rsp_cmd::TYPE, rsp::LENS_TYPE);
        break;
    case cmd::QUERY_RANGE:
        send_uart_frame(rsp_cmd::MIN_ZOOM, m_pZoom->get_min_zoom());
        send_uart_frame(rsp_cmd::MAX_ZOOM, m_pZoom->get_max_zoom());
        break;
    case cmd::QUERY_VERSION:
        send_uart_frame(rsp_cmd::VERSION, rsp::FW_VERSION);
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
           cmd_byte == cmd::ZOOM_INC ||
           cmd_byte == cmd::ZOOM_DEC ||
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

static void uart_clear_all_errors(UART_HandleTypeDef* huart) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
}

static void start_uart_dma_rx(UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, s_aRxBuf, sizeof(s_aRxBuf));
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return;
    }

    // Level 1: abort + clear errors + retry
    HAL_UART_AbortReceive(huart);
    HAL_DMA_Abort(huart->hdmarx);
    uart_clear_all_errors(huart);
    huart->RxState = HAL_UART_STATE_READY;
    huart->hdmarx->State = HAL_DMA_STATE_READY;

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, s_aRxBuf, sizeof(s_aRxBuf));
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return;
    }

    // Level 2: full UART DeInit/Init
    swo_printf("[COMM] UART recovery: full DeInit/Init\n");
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);
    uart_clear_all_errors(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_aRxBuf, sizeof(s_aRxBuf));
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART2) {
        swo_printf("[COMM] UART error: 0x%08lX\n", huart->ErrorCode);
        HAL_UART_AbortReceive(huart);
        HAL_DMA_Abort(huart->hdmarx);
        uart_clear_all_errors(huart);
        huart->RxState = HAL_UART_STATE_READY;
        huart->hdmarx->State = HAL_DMA_STATE_READY;
        start_uart_dma_rx(huart);
    }
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

    // Clear any pending UART errors from data received before DMA was armed
    // (e.g. upper computer sent data during MCU boot → ORE flag set)
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    huart2.ErrorCode = HAL_UART_ERROR_NONE;
    huart2.gState = HAL_UART_STATE_READY;
    huart2.RxState = HAL_UART_STATE_READY;

    // Start UART DMA receive (IDLE line detection)
    start_uart_dma_rx(&huart2);
    swo_printf("[COMM] Task started, UART DMA RX armed\n");

    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t iWatchdogCount = 0;
    uint16_t iLastDmaCndtr = 0;
    uint32_t iDmaStuckCount = 0;
    for (;;) {
        if (s_bRxReady) {
            s_bRxReady = false;
            iWatchdogCount = 0;
            iDmaStuckCount = 0;
            swo_printf("[COMM] RX %u bytes:", s_iRxLen);
            for (uint16_t i = 0; i < s_iRxLen && i < 8; i++) {
                swo_printf(" %02X", s_aRxBuf[i]);
            }
            swo_printf("\n");
            task.on_frame_received(s_aRxBuf, s_iRxLen);
            start_uart_dma_rx(&huart2);
        }

        // UART watchdog: runs every 1s (100 * 10ms)
        // Parasitic power from SWD prevents MCU reset on lens power-cycle,
        // so UART/DMA state can be corrupted without a proper POR.
        if (++iWatchdogCount >= 100) {
            iWatchdogCount = 0;

            // Check TX state: if gState stuck at BUSY_TX, transmits silently fail
            if (huart2.gState != HAL_UART_STATE_READY) {
                swo_printf("[COMM] Watchdog: gState=0x%02X, resetting TX\n",
                           huart2.gState);
                HAL_UART_AbortTransmit(&huart2);
                huart2.gState = HAL_UART_STATE_READY;
            }

            // Check RX: RxState not BUSY_RX → DMA not armed
            if (huart2.RxState != HAL_UART_STATE_BUSY_RX) {
                swo_printf("[COMM] Watchdog: RxState=0x%02X, restarting RX\n",
                           huart2.RxState);
                HAL_UART_AbortReceive(&huart2);
                HAL_DMA_Abort(huart2.hdmarx);
                start_uart_dma_rx(&huart2);
                iDmaStuckCount = 0;
            } else {
                // Check DMA stuck: CNDTR unchanged for 5s
                uint16_t iCndtr = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
                if (iCndtr == iLastDmaCndtr) {
                    iDmaStuckCount++;
                    if (iDmaStuckCount >= 5) {
                        swo_printf("[COMM] Watchdog: DMA stuck (CNDTR=%u), force restart\n",
                                   iCndtr);
                        HAL_UART_AbortReceive(&huart2);
                        HAL_DMA_Abort(huart2.hdmarx);
                        uart_clear_all_errors(&huart2);
                        huart2.RxState = HAL_UART_STATE_READY;
                        huart2.hdmarx->State = HAL_DMA_STATE_READY;
                        start_uart_dma_rx(&huart2);
                        iDmaStuckCount = 0;
                    }
                } else {
                    iDmaStuckCount = 0;
                }
                iLastDmaCndtr = iCndtr;
            }

        }

        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }
#else
    (void)params;
#endif
}
