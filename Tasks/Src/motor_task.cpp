// Tasks/Src/motor_task.cpp
#include "motor_task.hpp"
#include "motor_ctrl.hpp"
#include "encoder.hpp"
#include "stall_detect.hpp"

namespace zlens {

void motor_task_entry(void* params) {
    (void)params;
    // Task skeleton — full implementation after CubeMX integration
    // 1ms period: encoder read -> stall detect -> motor update -> queue check
}

} // namespace zlens
