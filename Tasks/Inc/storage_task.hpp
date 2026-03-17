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
    bool restore_params(FRAM_PARAMS_S& stParams);

    const FRAM_PARAMS_S& get_params() const { return m_stParams; }
    void set_params(const FRAM_PARAMS_S& stParams) { m_stParams = stParams; }

private:
    FramStorage* m_pFram = nullptr;
    QueueHandle_t m_saveQueue = nullptr;

    FRAM_PARAMS_S m_stParams = {};
    uint32_t m_iLastSaveTick = 0;
    int32_t m_iLastSavedPosition = 0;
    bool m_bParamsLoaded = false;

    void handle_save(const SAVE_MESSAGE_S& stMsg);
    void do_periodic_save();
    void write_params();
};

} // namespace zlens

extern "C" void storage_task_entry(void* params);
