# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

# 禁止在 build/ 根目录构建
get_filename_component(_build_dir_name "${CMAKE_BINARY_DIR}" NAME)
if(_build_dir_name STREQUAL "build")
    message(FATAL_ERROR
        "Do not use 'build/' directly. Use:\n"
        "  cmake -B build/fw  -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake\n"
        "  cmake -B build/test -DBUILD_TESTING=ON\n"
        "  cmake -B build/diag -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake -DBUILD_DIAG=ON")
endif()

option(BUILD_TESTING "Build unit tests for host" OFF)
option(BUILD_DIAG "Build diagnostic firmware" OFF)

if(BUILD_TESTING)
    project(ZLENS_DC_Test LANGUAGES C CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
    enable_testing()

    add_subdirectory(Test)
elseif(BUILD_DIAG)
    add_subdirectory(Test/diag)
else()
    project(ZLENS_DC LANGUAGES C CXX ASM)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    add_definitions(-DSTM32F103xE -DUSE_HAL_DRIVER)

    # Collect source files (CubeMX-generated + application)
    file(GLOB CORE_C_SOURCES   Core/Src/*.c)
    file(GLOB STARTUP_SOURCES  Core/Startup/*.s)
    file(GLOB HAL_SOURCES      Drivers/STM32F1xx_HAL_Driver/Src/*.c)
    file(GLOB APP_SOURCES      App/Src/*.cpp)
    file(GLOB TASK_SOURCES     Tasks/Src/*.cpp)

    # FreeRTOS middleware
    file(GLOB FREERTOS_CORE    Middlewares/Third_Party/FreeRTOS/Source/*.c)
    file(GLOB FREERTOS_CMSIS   Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c)
    file(GLOB FREERTOS_PORT    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c)
    set(FREERTOS_HEAP           Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c)

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
    )

    target_include_directories(${PROJECT_NAME}.elf PRIVATE
        Core/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
        App/Inc
        Tasks/Inc
    )

    option(PID_TUNE_LOG "Enable PID tuning SWO output" OFF)
    if(PID_TUNE_LOG)
        target_compile_definitions(${PROJECT_NAME}.elf PRIVATE PID_TUNE_LOG)
    endif()

    # Post-build: generate .hex/.bin and print size
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex  $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    )
endif()
