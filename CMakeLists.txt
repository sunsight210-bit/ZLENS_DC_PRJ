# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

option(BUILD_TESTING "Build unit tests for host" OFF)

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
else()
    project(ZLENS_DC LANGUAGES C CXX ASM)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    add_definitions(-DSTM32F103xE -DUSE_HAL_DRIVER)

    # Placeholder: Core/Drivers/App/Tasks sources added later
    # after CubeMX generation
    message(STATUS "ARM cross-compilation target — CubeMX sources required")
endif()
