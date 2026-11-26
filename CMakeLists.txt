cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_PROJECT_NAME cheesy_integrated)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX HAL + startup sources
add_subdirectory(cmake/stm32cubemx)

# All application sources in Core/Src
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/motor_api.c
    Core/Src/vl53l0x.c
    Core/Src/imu.c
    Core/Src/battery.c
    Core/Src/ble_telemetry.c
    Core/Src/ram_monitor.c
    Core/Src/runtime_config.c
    Core/Src/app.c
    Core/Src/robot_hw_bridge.c
    Core/Src/ekf_localization.c
    Core/Src/arena_map.c
    Core/Src/obstacle_mapper.c
    Core/Src/astar_planner.c
    Core/Src/pure_pursuit.c
    Core/Src/mission_fsm.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# Post-build: Generate ready-to-flash .bin & .hex images and print Flash/RAM footprint
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating flashable ${CMAKE_PROJECT_NAME}.bin / .hex and size report"
)
