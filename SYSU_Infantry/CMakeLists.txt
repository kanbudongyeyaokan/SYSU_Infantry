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

file(GLOB UI_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/UI/*.c"
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        1-Hardware_Layer/bsp_dwt/bsp_dwt.h
        1-Hardware_Layer/bsp_dwt/bsp_dwt.c
        3-Function_Module_Layer/Remote_Control/remote_control.h
        3-Function_Module_Layer/Remote_Control/remote_control.c
        3-Function_Module_Layer/Remote_Control/SBUS.h
        3-Function_Module_Layer/Remote_Control/SBUS.c
        3-Function_Module_Layer/Remote_Control/video_link.h
        3-Function_Module_Layer/Remote_Control/video_link.c
        1-Hardware_Layer/bsp_usart/bsp_usart.c
        1-Hardware_Layer/bsp_usart/bsp_usart.h
        1-Hardware_Layer/bsp_rtt/bsp_rtt.h
        1-Hardware_Layer/bsp_rtt/bsp_rtt.c
        0-Middleware_Layer/Message_Center/message_center.c
        0-Middleware_Layer/Message_Center/message_center.h
        0-Middleware_Layer/Math_lib/math_lib.h
        1-Hardware_Layer/bsp_can/bsp_can.c
        1-Hardware_Layer/bsp_can/bsp_can.h
        1-Hardware_Layer/bsp_wdg/bsp_wdg.c
        1-Hardware_Layer/bsp_wdg/bsp_wdg.h
        4-Application_Layer/Robot_Task/robot_task.h
        4-Application_Layer/Robot_Task/robot_task.c
        4-Application_Layer/Execution/Chassis_Task/Chassis_task.h
        4-Application_Layer/Execution/Chassis_Task/Chassis_task.c
        4-Application_Layer/Execution/Gimbal_Task/Gimbal_task.c
        4-Application_Layer/Execution/Gimbal_Task/Gimbal_task.h
        4-Application_Layer/Perception/Ins_Task/Ins_task.c
        4-Application_Layer/Perception/Ins_Task/Ins_task.h
        4-Application_Layer/Perception/Referee_Task/Referee_task.c
        4-Application_Layer/Perception/Referee_Task/Referee_task.h
        4-Application_Layer/Execution/Shoot_Task/shoot_task.c
        4-Application_Layer/Execution/Shoot_Task/shoot_task.h
        4-Application_Layer/Execution/Motor_Task/motor_task.c
        4-Application_Layer/Execution/Motor_Task/motor_task.h
        4-Application_Layer/Execution/Watchdog_Task/watchdog_task.h
        4-Application_Layer/Execution/Watchdog_Task/watchdog_task.c

        0-Middleware_Layer/Error_System/inc/error_handler.h
        0-Middleware_Layer/Error_System/src/error_handler.c
        0-Middleware_Layer/Error_System/src/error_port.c


        2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver/bmi088.h
        2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver/bmi088.c
        2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver/bmi088_reg_def.h
        2-Hardware_Driver_Layer/IMU_Driver/Hwt606_Driver/hwt606_uart.c
        2-Hardware_Driver_Layer/IMU_Driver/Hwt606_Driver/hwt606_uart.h
        2-Hardware_Driver_Layer/IMU_Driver/Hwt606_Driver/hwt606_iic.c
        2-Hardware_Driver_Layer/IMU_Driver/Hwt606_Driver/hwt606_iic.h
        2-Hardware_Driver_Layer/IMU_Driver/Hwt101_Driver/hwt101_iic.c
        2-Hardware_Driver_Layer/IMU_Driver/Hwt101_Driver/hwt101_iic.h

        
        0-Middleware_Layer/Pid_Controller/algorithm_pid.h
        0-Middleware_Layer/Pid_Controller/algorithm_pid.c
        0-Middleware_Layer/Kalman_Fliter/algorithm_ekf.h
        0-Middleware_Layer/Kalman_Fliter/algorithm_ekf.c
        0-Middleware_Layer/Kalman_Fliter/algorithm_kf.h
        0-Middleware_Layer/Kalman_Fliter/algorithm_kf.c


        0-Middleware_Layer/letter-shell-master/src/shell.c
        0-Middleware_Layer/letter-shell-master/src/shell.h
        0-Middleware_Layer/letter-shell-master/src/shell_cfg.h
        0-Middleware_Layer/letter-shell-master/src/shell_cmd_list.c
        0-Middleware_Layer/letter-shell-master/src/shell_companion.c
        0-Middleware_Layer/letter-shell-master/src/shell_ext.c
        0-Middleware_Layer/letter-shell-master/src/shell_ext.h

        0-Middleware_Layer/SystemView-main/SEGGER/SEGGER.h
        0-Middleware_Layer/SystemView-main/SYSVIEW/SEGGER_SYSVIEW.c
        0-Middleware_Layer/SystemView-main/SYSVIEW/SEGGER_SYSVIEW.h
        0-Middleware_Layer/SystemView-main/SYSVIEW/SEGGER_SYSVIEW_ConfDefaults.h
        0-Middleware_Layer/SystemView-main/SYSVIEW/SEGGER_SYSVIEW_Int.h

        0-Middleware_Layer/SystemView-main/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.c
        0-Middleware_Layer/SystemView-main/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.h
        0-Middleware_Layer/SystemView-main/Config/SEGGER_SYSVIEW_Conf.h
        0-Middleware_Layer/SystemView-main/Config/Global.h

        0-Middleware_Layer/RTT-main/RTT/SEGGER_RTT.c
        0-Middleware_Layer/RTT-main/RTT/SEGGER_RTT.h
        0-Middleware_Layer/RTT-main/RTT/SEGGER_RTT_ASM_ARMv7M.S
        0-Middleware_Layer/RTT-main/RTT/SEGGER_RTT_printf.c
        0-Middleware_Layer/RTT-main/Config/SEGGER_RTT_Conf.h

        4-Application_Layer/For_Testing_Task/can_motors_test_task.h
        4-Application_Layer/For_Testing_Task/can_motors_test_task.c
        4-Application_Layer/For_Testing_Task/rc_test_task.h
        4-Application_Layer/For_Testing_Task/rc_test_task.c
        4-Application_Layer/For_Testing_Task/message_test_task.h
        4-Application_Layer/For_Testing_Task/message_test_task.c
        4-Application_Layer/For_Testing_Task/chassis_motor_integration_test.c
        4-Application_Layer/For_Testing_Task/chassis_motor_integration_test.h
        4-Application_Layer/Decision_Making/Decision_making_task.c
        4-Application_Layer/Decision_Making/Decision_making_task.h
        4-Application_Layer/Decision_Making/Shell_task.c
        4-Application_Layer/Decision_Making/Shell_task.h

        2-Hardware_Driver_Layer/Motor_Driver/dji_motor.c
        2-Hardware_Driver_Layer/Motor_Driver/dji_motor.h

        3-Function_Module_Layer/Robot_Definitions/robot_definitions.h
        3-Function_Module_Layer/Chassis/chassis.c
        3-Function_Module_Layer/Chassis/chassis.h
        3-Function_Module_Layer/Chassis/chassis_ramp.h
        3-Function_Module_Layer/Chassis/chassis_ramp.c

        3-Function_Module_Layer/Chassis/chassis_power_control.c
        3-Function_Module_Layer/Chassis/chassis_power_control.h
        3-Function_Module_Layer/Chassis/chassis_scurve.c
        3-Function_Module_Layer/Chassis/chassis_scurve.h
        
        3-Function_Module_Layer/Gimbal/gimbal.c
        3-Function_Module_Layer/Gimbal/gimbal.h
        3-Function_Module_Layer/Ins/ins.c
        3-Function_Module_Layer/Ins/ins.h
        3-Function_Module_Layer/Referee/referee.c
        3-Function_Module_Layer/Referee/referee.h
        3-Function_Module_Layer/Referee/referee_ui.h
        3-Function_Module_Layer/Referee/referee_ui.c

        3-Function_Module_Layer/Shoot/shoot.c
        3-Function_Module_Layer/Shoot/shoot.h
        3-Function_Module_Layer/Shoot/shoot_heat_control.c
        3-Function_Module_Layer/Shoot/shoot_heat_control.h
        3-Function_Module_Layer/Super_Capacitor/supercap_comm.c
        3-Function_Module_Layer/Super_Capacitor/supercap_comm.h
        3-Function_Module_Layer/Power_Meter/power_meter.c
        3-Function_Module_Layer/Power_Meter/power_meter.h
        3-Function_Module_Layer/Ins/ins.c
        3-Function_Module_Layer/Ins/ins.h


        3-Function_Module_Layer/Decision_making/decision_making.c
        3-Function_Module_Layer/Decision_making/decision_making.h
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_driver.h
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_driver.c
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_music.h
        2-Hardware_Driver_Layer/Buzzer_Driver/buzzer_music.c

        2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver/bmi088_temp.h
        2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver/bmi088_temp.c

        0-Middleware_Layer/Kalman_Fliter/MahonyAHRS.c
        0-Middleware_Layer/Kalman_Fliter/MahonyAHRS.h
        3-Function_Module_Layer/Vofa/vofa.c
        3-Function_Module_Layer/Vofa/vofa.h
        4-Application_Layer/Decision_Making/Shell_task.c
        4-Application_Layer/Decision_Making/Shell_task.h
        3-Function_Module_Layer/Shell/shell_port.c
        3-Function_Module_Layer/Shell/shell_port.h

        3-Function_Module_Layer/Vision_Comm/vision_comm.c
        3-Function_Module_Layer/Vision_Comm/vision_comm.h

        0-Middleware_Layer/Crc/crc_referee.c
        0-Middleware_Layer/Crc/crc_referee.h


        0-Middleware_Layer/Ramp_Controller/ramp_controller.c
        0-Middleware_Layer/Ramp_Controller/ramp_controller.h
        0-Middleware_Layer/Filter/lowpass_filter.c
        0-Middleware_Layer/Filter/lowpass_filter.h

        0-Middleware_Layer/Error_System/inc/error_handler.h
        0-Middleware_Layer/Error_System/src/error_handler.c
        0-Middleware_Layer/Error_System/src/error_port.c

        1-Hardware_Layer/bsp_usb/bsp_usb.c
        1-Hardware_Layer/bsp_usb/bsp_usb.h
        1-Hardware_Layer/bsp_spi/bsp_spi.c
        1-Hardware_Layer/bsp_spi/bsp_spi.h
        1-Hardware_Layer/bsp_iic/bsp_iic.c
        1-Hardware_Layer/bsp_iic/bsp_iic.h
        3-Function_Module_Layer/Buzzer_Alarm/buzzer_alarm.c
        3-Function_Module_Layer/Buzzer_Alarm/buzzer_alarm.h
        4-Application_Layer/Buzzer_Alarm_Task/buzzer_alarm_task.c
        4-Application_Layer/Buzzer_Alarm_Task/buzzer_alarm_task.h

)


# Add STM32CubeMX generated sources
# =========================================================================
# 👇👇👇 暴力解封 C板 (STM32F407) 算力与硬件浮点 👇👇👇
# =========================================================================

# # 1. 强制添加编译选项：开启 O2 优化，启用 Cortex-M4 硬浮点指令集
# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
#         -O2
#         -mfloat-abi=hard
#         -mfpu=fpv4-sp-d16
# )

# # 2. 强制添加链接选项：告诉链接器使用硬浮点，防止报错
# target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
#         -mfloat-abi=hard
#         -mfpu=fpv4-sp-d16
# )

# # =========================================================================
# # 👆👆👆 插入结束 👆👆👆
# # =========================================================================
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
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Remote_Control
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Decision_making
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_can
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_wdg
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Pid_Controller
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Message_Center
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Kalman_Fliter
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Math_lib
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Error_System/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Crc
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/UI

        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/letter-shell-master/src

        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/SEGGER
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/SYSVIEW
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/Config
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/Sample
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/Sample/FreeRTOSV10
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/Sample/FreeRTOSV10/Patch
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/SystemView-main/Sample/FreeRTOSV10/Config

        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/RTT-main/Config
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/RTT-main/RTT

        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Ramp_Controller
        ${CMAKE_CURRENT_SOURCE_DIR}/0-Middleware_Layer/Filter
                ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/IMU_Driver/HWT101_Driver

        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Robot_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/For_Testing_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/Motor_Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Super_Capacitor
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Power_Meter

        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_usb
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_spi
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_iic
        ${CMAKE_CURRENT_SOURCE_DIR}/1-Hardware_Layer/bsp_rtt


        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/Buzzer_Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Execution/Chassis_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Execution/Gimbal_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Execution/Motor_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Perception/Ins_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Perception/Referee_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Remote_Control
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Execution/Shoot_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Execution/Watchdog_Task
        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/IMU_Driver/Bmi088_Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/IMU_Driver/Hwt606_Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/2-Hardware_Driver_Layer/IMU_Driver/Hwt101_Driver
        
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Decision_Making
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Robot_Definitions
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Vision_Comm


        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Ins
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Referee
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Shoot
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Remote_Control
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Vofa
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Buzzer_Alarm
        ${CMAKE_CURRENT_SOURCE_DIR}/3-Function_Module_Layer/Shell
        ${CMAKE_CURRENT_SOURCE_DIR}/4-Application_Layer/Buzzer_Alarm_Task

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
