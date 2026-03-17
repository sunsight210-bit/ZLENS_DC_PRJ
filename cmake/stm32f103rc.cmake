# cmake/stm32f103rc.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(TOOLCHAIN_DIR "/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_C_FLAGS_INIT   "${MCU_FLAGS} -fdata-sections -ffunction-sections -Wall")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti -Wall -std=c++17")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/cmake/stm32f1xx.ld")
