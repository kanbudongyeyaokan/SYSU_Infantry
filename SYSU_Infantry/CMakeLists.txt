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
set(CMAKE_PROJECT_NAME SYSU_Infantry)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        1-Hardware_Layer/bsp_dwt/bsp_dwt.h
        1-Hardware_Layer/bsp_dwt/bsp_dwt.c
        3-Function_Module_Layer/Remote_Control/remote_control.h
        3-Function_Module_Layer/Remote_Control/remote_control.c
        1-Hardware_Layer/bsp_usart/bsp_usart.c
        1-Hardware_Layer/bsp_usart/bsp_usart.h
        0-Middleware_Layer/Message_Center/message_center.c
        0-Middleware_Layer/Message_Center/message_center.h
        1-Hardware_Layer/bsp_can/bsp_can.c
        1-Hardware_Layer/bsp_can/bsp_can.h
        4-Application_Layer/Robot_Task/robot_task.h
        4-Application_Layer/Robot_Task/robot_task.c
        4-Application_Layer/Chassis/Chassis_task.h
        4-Application_Layer/Chassis/Chassis_task.c
        4-Application_Layer/Gimbal/Gimbal_task.c
        4-Application_Layer/Gimbal/Gimbal_task.h
        4-Application_Layer/Ins/Ins_task.c
        4-Application_Layer/Ins/Ins_task.h
        4-Application_Layer/Others/Others_task.c
        4-Application_Layer/Others/Others_task.h
        4-Application_Layer/Referee/Referee_task.c
        4-Application_Layer/Referee/Referee_task.h
        4-Application_Layer/Remote_Control/rc_task.c
        4-Application_Layer/Remote_Control/rc_task.h
        4-Application_Layer/Shoot/shoot_task.c
        4-Application_Layer/Shoot/shoot_task.h
        2-Hardware_Driver_Layer/Motor_Driver/dji_motor.h
        2-Hardware_Driver_Layer/Motor_Driver/dji_motor.c
        0-Middleware_Layer/Pid_Controller/algorithm_pid.h
        0-Middleware_Layer/Pid_Controller/algorithm_pid.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_usart
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_dwt
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Remote_Control
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_can
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Pid_Controller
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
