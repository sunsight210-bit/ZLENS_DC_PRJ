// Tasks/Src/comm_task.cpp
#include "comm_task.hpp"
#include "comm_protocol.hpp"

namespace zlens {

void comm_task_entry(void* params) {
    (void)params;
    // Task skeleton — full implementation after CubeMX integration
    // Event-driven: UART IDLE interrupt -> frame parse -> cmd queue -> rsp queue -> UART TX
}

} // namespace zlens
