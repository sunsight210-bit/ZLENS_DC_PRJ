// Tasks/Src/storage_task.cpp
#include "storage_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#endif

namespace zlens {

void StorageTask::init(FramStorage* pFram, QueueHandle_t saveQ) {
    m_pFram = pFram;
    m_saveQueue = saveQ;
    m_iLastSaveTick = HAL_GetTick();
    m_iLastSavedPosition = 0;
    m_bStateLoaded = false;
    m_stState = {};
}

void StorageTask::run_once() {
    SAVE_MESSAGE_S stMsg;
    if (xQueueReceive(m_saveQueue, &stMsg, 0) == pdTRUE) {
        handle_save(stMsg);
    }
    do_periodic_save();
}

bool StorageTask::restore_state(FRAM_STATE_S& stState) {
    if (!m_pFram->load_state(m_stState)) {
        return false;
    }
    stState = m_stState;
    m_iLastSavedPosition = m_stState.current_position;
    m_bStateLoaded = true;
    return true;
}

void StorageTask::handle_save(const SAVE_MESSAGE_S& stMsg) {
    if (g_bSpiEmergency) return;

    m_stState.current_position = stMsg.position;
    m_stState.current_zoom_x10 = stMsg.zoom_x10;
    m_stState.last_save_reason = stMsg.reason;

    if (stMsg.homing_done == 1) {
        m_stState.homing_done = 1;
        m_stState.move_count = 0;
    }
    if (stMsg.position_valid == 0xFF) {
        m_stState.position_valid = 0xFF;
        m_stState.move_count++;
    }

    write_state();
    m_iLastSavedPosition = stMsg.position;
    m_iLastSaveTick = HAL_GetTick();
}

void StorageTask::do_periodic_save() {
    uint32_t iTick = HAL_GetTick();
    if (iTick - m_iLastSaveTick < PERIODIC_SAVE_INTERVAL_MS) return;
    if (g_bSpiEmergency) return;

    if (m_stState.current_position != m_iLastSavedPosition) {
        m_stState.last_save_reason = save_reason::PERIODIC;
        write_state();
        m_iLastSavedPosition = m_stState.current_position;
    }
    m_iLastSaveTick = iTick;
}

void StorageTask::write_state() {
    m_stState.magic = FramStorage::MAGIC;
    m_stState.crc16 = FramStorage::calc_crc(m_stState);
    m_pFram->save_state(m_stState);
}

} // namespace zlens

extern "C" void storage_task_entry(void* params) {
#ifndef BUILD_TESTING
    (void)params;
    using namespace zlens;

    static StorageTask task;
    task.init(&g_FramStorage, g_saveQueue);

    // Restore saved state on boot
    FRAM_STATE_S stState;
    if (task.restore_state(stState)) {
        g_Encoder.set_position(stState.current_position);
    }

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }
#else
    (void)params;
#endif
}
