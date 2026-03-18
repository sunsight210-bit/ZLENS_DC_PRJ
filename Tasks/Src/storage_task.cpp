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
    m_bParamsLoaded = false;
    m_stParams = {};
}

void StorageTask::run_once() {
    // Check save queue (non-blocking)
    SAVE_MESSAGE_S stMsg;
    if (xQueueReceive(m_saveQueue, &stMsg, 0) == pdTRUE) {
        handle_save(stMsg);
    }

    // Periodic save check
    do_periodic_save();
}

bool StorageTask::restore_params(FRAM_PARAMS_S& stParams) {
    if (!m_pFram->load_params(m_stParams)) {
        return false;
    }
    if (!FramStorage::verify_crc(m_stParams) || !FramStorage::check_magic(m_stParams)) {
        return false;
    }
    stParams = m_stParams;
    m_iLastSavedPosition = m_stParams.current_position;
    m_bParamsLoaded = true;
    return true;
}

void StorageTask::handle_save(const SAVE_MESSAGE_S& stMsg) {
    if (g_bSpiEmergency) return;

    m_stParams.current_position = stMsg.position;
    m_stParams.current_zoom_x10 = stMsg.zoom_x10;
    m_stParams.last_save_reason = stMsg.reason;

    write_params();
    m_iLastSavedPosition = stMsg.position;
    m_iLastSaveTick = HAL_GetTick();
}

void StorageTask::do_periodic_save() {
    uint32_t iTick = HAL_GetTick();
    if (iTick - m_iLastSaveTick < PERIODIC_SAVE_INTERVAL_MS) return;
    if (g_bSpiEmergency) return;

    // Only save if position changed
    if (m_stParams.current_position != m_iLastSavedPosition) {
        m_stParams.last_save_reason = save_reason::PERIODIC;
        write_params();
        m_iLastSavedPosition = m_stParams.current_position;
    }
    m_iLastSaveTick = iTick;
}

void StorageTask::write_params() {
    m_stParams.magic_number = FramStorage::MAGIC;
    m_stParams.version = 1;
    m_stParams.crc16 = FramStorage::calc_crc(m_stParams);
    m_pFram->save_params(m_stParams);
}

} // namespace zlens

extern "C" void storage_task_entry(void* params) {
#ifndef BUILD_TESTING
    (void)params;
    using namespace zlens;

    static StorageTask task;
    task.init(&g_FramStorage, g_saveQueue);

    // Restore saved parameters on boot
    FRAM_PARAMS_S stParams;
    if (task.restore_params(stParams)) {
        g_Encoder.set_position(stParams.current_position);
        g_ZoomTable.set_total_range(stParams.total_range);
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
