cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RTOS_IG)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR} Applications/Src/CAN_receive.c
    ${CMAKE_SOURCE_DIR} Applications/Src/pid.c
    ${CMAKE_SOURCE_DIR} Applications/Src/pid_user.c
    ${CMAKE_SOURCE_DIR} Applications/Src/swerve_ctrl.c
    ${CMAKE_SOURCE_DIR} Applications/Src/meth_ctrl.c
    ${CMAKE_SOURCE_DIR} Applications/Src/meth_driver.c
    ${CMAKE_SOURCE_DIR} Applications/Src/servo.c
    ${CMAKE_SOURCE_DIR} Applications/Src/sbus.c
    ${CMAKE_SOURCE_DIR} Applications/Src/recv_send.c

    ${CMAKE_SOURCE_DIR} Math/Src/math_calc.c
    ${CMAKE_SOURCE_DIR} Math/Src/tool_calc.c

    ${CMAKE_SOURCE_DIR} Bsp/Src/bsp_can.c

    ${CMAKE_SOURCE_DIR} IMU/Src/bmi088.c
    ${CMAKE_SOURCE_DIR} IMU/Src/ist8310.c
    ${CMAKE_SOURCE_DIR} IMU/Src/imu_fusion.c
    ${CMAKE_SOURCE_DIR} IMU/Src/Yaw_hold.c

    ${CMAKE_SOURCE_DIR} Task/Src/task_can.c
    ${CMAKE_SOURCE_DIR} Task/Src/task_meth.c
    ${CMAKE_SOURCE_DIR} Task/Src/task_remote.c
    ${CMAKE_SOURCE_DIR} Task/Src/task_swerve.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR} Applications/Inc
    ${CMAKE_SOURCE_DIR} Math/Inc
    ${CMAKE_SOURCE_DIR} Bsp/Inc
    ${CMAKE_SOURCE_DIR} Driver/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR} IMU/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
