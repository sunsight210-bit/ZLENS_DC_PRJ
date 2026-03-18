# Test/diag/CMakeLists.txt — Independent ARM diagnostic firmware
# Build: cmake -B build/diag -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake -DBUILD_DIAG=ON
# Output: build/diag/ZLENS_DIAG.elf

project(ZLENS_DIAG LANGUAGES C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DSTM32F103xE -DUSE_HAL_DRIVER)

# Reuse product sources (Core, HAL, FreeRTOS, App)
set(SRC_ROOT "${CMAKE_SOURCE_DIR}")

file(GLOB CORE_C_SOURCES   ${SRC_ROOT}/Core/Src/*.c)
file(GLOB STARTUP_SOURCES  ${SRC_ROOT}/Core/Startup/*.s)
file(GLOB HAL_SOURCES      ${SRC_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/*.c)
file(GLOB APP_SOURCES      ${SRC_ROOT}/App/Src/*.cpp)
file(GLOB TASK_SOURCES     ${SRC_ROOT}/Tasks/Src/*.cpp)

# FreeRTOS
file(GLOB FREERTOS_CORE    ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/*.c)
file(GLOB FREERTOS_CMSIS   ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c)
file(GLOB FREERTOS_PORT    ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c)
set(FREERTOS_HEAP           ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c)

# Remove product app_instances.cpp — diag provides its own app_init()
list(FILTER APP_SOURCES EXCLUDE REGEX "app_instances\\.cpp$")

add_executable(${PROJECT_NAME}.elf
    ${CORE_C_SOURCES}
    ${STARTUP_SOURCES}
    ${HAL_SOURCES}
    ${APP_SOURCES}
    ${TASK_SOURCES}
    ${FREERTOS_CORE}
    ${FREERTOS_CMSIS}
    ${FREERTOS_PORT}
    ${FREERTOS_HEAP}
    ${CMAKE_CURRENT_SOURCE_DIR}/phase3_motor_diag.cpp
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${SRC_ROOT}/Core/Inc
    ${SRC_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${SRC_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${SRC_ROOT}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${SRC_ROOT}/Drivers/CMSIS/Include
    ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${SRC_ROOT}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
    ${SRC_ROOT}/App/Inc
    ${SRC_ROOT}/Tasks/Inc
)

# Post-build: generate .hex/.bin and print size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex  $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)
