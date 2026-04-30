# Agent Guide for SYSU_Infantry

## Project Overview
This is a RoboMaster infantry robot control system built for the 2026 season using STM32F407 microcontroller with FreeRTOS RTOS. The system controls the chassis, gimbal, and shooting mechanisms for competition robotics.

## Build Commands
- Initial build: `cmake --build build/ --config Debug` (after configuring with CMake)
- Clean rebuild: `rm -rf build && mkdir build && cd build && cmake .. && make`
- Flash to device: `./flash.sh`

## Lint/Test Commands
- No dedicated testing/linting commands appear to exist in the project.
- Manual inspection of C files needed for verification.
- Static analysis could be performed using tools like PC-lint or cppcheck.

## Code Style Guidelines

### Imports
- Include necessary headers based on functionality dependencies in layers.
- Standard headers (like `stdio.h`, `string.h`) come first.
- CMSIS and HAL headers follow.
- Custom headers from the project come last.

### Formatting
- Use consistent indentation (mixed tabs/spaces in some files, but aim for 4 spaces).
- Lines should be reasonable length (under 120 chars preferred).
- Follow K&R style braces with new-line after opening brace.
- No additional comments beyond essential function/file headers.

### Naming Conventions
- Type definitions: `snake_case_t` with `_t` suffix (e.g. `Shoot_cmd_send_t`).
- Enums: `snake_case_e` with `_e` suffix (e.g. `shoot_mode_e`).
- Variables: `snake_case` (e.g. `shoot_cmd_recv`).
- Functions: `snake_case` (e.g. `Shoot_handle_command`).
- Constants/macros: `UPPER_CASE` with `#define` (e.g. `ONE_BULLET_DELTA_ANGLE`).
- Struct members: `snake_case` (consistent with general variables).

### Types
- Use specific typedefs for structures like `Uart_instance_t`, `QueueHandle_t`, `Djimotor_device_t`.
- Prefer standard integer types where possible (`uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, etc).
- Boolean types: `bool` from stdbool.h.
- Floating point: Generally `float` (32-bit) for embedded constraints.

### Error Handling
- Use the custom error system with macros: `ERROR_INFO`, `ERROR_WARN`, `ERROR_RAISE`, `ERROR_CRITICAL`.
- Report errors with modules like `"SHOOT"` and descriptive messages.
- Integrate with UART output using `test_uart` variable.
- Check return values for FreeRTOS functions and CAN communications.

## Layer Architecture
Project follows a layered architecture:
- 0-Middleware_Layer: PID controllers, Kalman filters, error systems, message center
- 1-Hardware_Layer: Hardware abstraction drivers (CAN, USART, DWT timer)
- 2-Hardware_Driver_Layer: Concrete hardware drivers (DJIMotor, IMU)
- 3-Function_Module_Layer: Functional modules (Gimbal, Chassis, Shoot logic)
- 4-Application_Layer: Top-level task coordination

## Special Considerations
- Embedded real-time constraints apply to all operations.
- Motor control operates at 1000Hz in the `Motor_control_task`.
- CAN communication is primary inter-device bus connecting motors.
- Careful power management is important for battery operated robot.
- Safety mechanisms include watchdog and emergency stop routines.
- Fixed-point arithmetic preferred over floating point when possible.
- Memory allocation: Avoid dynamic allocation during runtime in control loops.

## Build Process Instructions
- When automatically building and testing the code, if there is a compilation error, DO NOT attempt to fix it automatically. Instead, stop and present the compilation errors to the user for review.
