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
        1-Hardware_Layer/bsp_usart/bsp_usart.c
        1-Hardware_Layer/bsp_usart/bsp_usart.h
        1-Hardware_Layer/bsp_rtt/bsp_rtt.h
        1-Hardware_Layer/bsp_rtt/bsp_rtt.c
        1-Hardware_Layer/bsp_can/bsp_can.c
        1-Hardware_Layer/bsp_can/bsp_can.h
        4-Application_Layer/Robot_Task/robot_task.h
        4-Application_Layer/Robot_Task/robot_task.c

        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_driver.h
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_driver.c
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_music.h
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_music.c





        0-Middleware_Layer/Error_System/inc/error_handler.h
        0-Middleware_Layer/Error_System/src/error_handler.c
        0-Middleware_Layer/Error_System/src/error_port.c

        1-Hardware_Layer/bsp_usb/bsp_usb.c
        1-Hardware_Layer/bsp_usb/bsp_usb.h
        1-Hardware_Layer/bsp_spi/bsp_spi.c
        1-Hardware_Layer/bsp_spi/bsp_spi.h

        3-Function_Module_Layer/Buzzer_Alarm/buzzer_alarm.c
        3-Function_Module_Layer/Buzzer_Alarm/buzzer_alarm.h
        4-Application_Layer/Buzzer_Alarm_Task/buzzer_alarm_task.c
        4-Application_Layer/Buzzer_Alarm_Task/buzzer_alarm_task.h
        0-Middleware_Layer/RTT-main/RTT/SEGGER_RTT.c
        3-Function_Module_Layer/Bridge_Link/bridge_link.c
        3-Function_Module_Layer/Bridge_Link/bridge_link_task.c

)

add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${UI_SOURCES}
        # Add user sources here
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Robot_Task/robot_task.c
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_usart
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_dwt
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_can
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Error_System/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/RTT-main/Config
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/RTT-main/RTT
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Robot_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_usb
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_spi
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_rtt
        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/Buzzer_Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Buzzer_Alarm
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Buzzer_Alarm_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Bridge_Link

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        __FPU_PRESENT=1
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
                stm32cubemx

                # Add user defined libraries
                m
)
