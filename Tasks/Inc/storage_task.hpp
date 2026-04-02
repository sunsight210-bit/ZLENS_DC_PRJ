// Tasks/Inc/storage_task.hpp
#pragma once
#include "task_config.hpp"
#include "fram_storage.hpp"

namespace zlens {

class StorageTask {
public:
    static constexpr uint32_t PERIODIC_SAVE_INTERVAL_MS = 500;

    void init(FramStorage* pFram, QueueHandle_t saveQ);
    void run_once();
    bool restore_state(FRAM_STATE_S& stState);

    const FRAM_STATE_S& get_state() const { return m_stState; }
    void set_state(const FRAM_STATE_S& stState) { m_stState = stState; }

private:
    FramStorage* m_pFram = nullptr;
    QueueHandle_t m_saveQueue = nullptr;

    FRAM_STATE_S m_stState = {};
    uint32_t m_iLastSaveTick = 0;
    int32_t m_iLastSavedPosition = 0;
    bool m_bStateLoaded = false;

    void handle_save(const SAVE_MESSAGE_S& stMsg);
    void do_periodic_save();
    void write_state();
};

} // namespace zlens

extern "C" void storage_task_entry(void* params);
