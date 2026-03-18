# Test/CMakeLists.txt
set(MOCK_SOURCES
    mocks/mock_hal.cpp
    mocks/mock_freertos.cpp
)

# Mock library shared by all tests
add_library(zlens_mocks STATIC ${MOCK_SOURCES})
target_include_directories(zlens_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Tasks/Inc
)
target_compile_definitions(zlens_mocks PUBLIC BUILD_TESTING)

# Helper function to add a test
function(zlens_add_test NAME SOURCE APP_SOURCES)
    add_executable(${NAME} ${SOURCE} ${APP_SOURCES} ${ARGN})
    target_link_libraries(${NAME} PRIVATE zlens_mocks GTest::gtest_main)
    target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        ${CMAKE_SOURCE_DIR}/App/Inc
        ${CMAKE_SOURCE_DIR}/Tasks/Inc
    )
    target_compile_definitions(${NAME} PRIVATE BUILD_TESTING)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

# --- CRC16 ---
add_executable(test_crc16 test_crc16.cpp ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_crc16 PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_crc16 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_crc16 COMMAND test_crc16)

# --- CommProtocol ---
add_executable(test_comm_protocol test_comm_protocol.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/comm_protocol.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_comm_protocol PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_comm_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_comm_protocol COMMAND test_comm_protocol)

# --- Encoder ---
add_executable(test_encoder test_encoder.cpp ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp)
target_link_libraries(test_encoder PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_encoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_encoder COMMAND test_encoder)

# --- ZoomTable ---
add_executable(test_zoom_table test_zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_zoom_table PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_zoom_table PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_zoom_table COMMAND test_zoom_table)

# --- FramStorage ---
add_executable(test_fram_storage test_fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp)
target_link_libraries(test_fram_storage PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_fram_storage PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_fram_storage COMMAND test_fram_storage)

# --- AdcFilter ---
add_executable(test_adc_filter test_adc_filter.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/adc_filter.cpp)
target_link_libraries(test_adc_filter PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_adc_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_adc_filter COMMAND test_adc_filter)

# --- StallDetect ---
add_executable(test_stall_detect test_stall_detect.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/stall_detect.cpp)
target_link_libraries(test_stall_detect PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_stall_detect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_stall_detect COMMAND test_stall_detect)

# --- MotorCtrl ---
add_executable(test_motor_ctrl test_motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp)
target_link_libraries(test_motor_ctrl PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_motor_ctrl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_motor_ctrl COMMAND test_motor_ctrl)

# --- PowerMonitor ---
add_executable(test_power_monitor test_power_monitor.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/power_monitor.cpp)
target_link_libraries(test_power_monitor PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_power_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_power_monitor COMMAND test_power_monitor)

# --- SystemManager ---
add_executable(test_system_manager test_system_manager.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/system_manager.cpp)
target_link_libraries(test_system_manager PRIVATE zlens_mocks GTest::gtest_main)
target_include_directories(test_system_manager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks ${CMAKE_SOURCE_DIR}/App/Inc)
add_test(NAME test_system_manager COMMAND test_system_manager)

# ==========================================
# Task-level tests (Phase 3-6)
# ==========================================

# --- MotorTask ---
zlens_add_test(test_motor_task test_motor_task.cpp
    ${CMAKE_SOURCE_DIR}/Tasks/Src/motor_task.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/stall_detect.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/adc_filter.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
)

# --- CommTask ---
zlens_add_test(test_comm_task test_comm_task.cpp
    ${CMAKE_SOURCE_DIR}/Tasks/Src/comm_task.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/comm_protocol.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/system_manager.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp
)

# --- StorageTask ---
zlens_add_test(test_storage_task test_storage_task.cpp
    ${CMAKE_SOURCE_DIR}/Tasks/Src/storage_task.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp
)

# --- SelfTest ---
zlens_add_test(test_self_test test_self_test.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/self_test.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/power_monitor.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/stall_detect.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp
)

# --- MonitorTask ---
zlens_add_test(test_monitor_task test_monitor_task.cpp
    ${CMAKE_SOURCE_DIR}/Tasks/Src/monitor_task.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/system_manager.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/power_monitor.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/fram_storage.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/self_test.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/encoder.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/motor_ctrl.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/stall_detect.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/zoom_table.cpp
    ${CMAKE_SOURCE_DIR}/App/Src/crc16.cpp
)
