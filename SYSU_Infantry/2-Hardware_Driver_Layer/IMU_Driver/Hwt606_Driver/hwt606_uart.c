// #include "hwt606_uart.h"
// #include <string.h>
//
// // 静态实例
// static HWT606_Data_t hwt_data;
// static Uart_instance_t *hwt_uart_instance = NULL;
//
// /**
//  * @brief 内部函数：处理单个角度数据包
//  * @param packet 指向以0x55开始的11字节数组
//  */
// static void Process_Angle_Packet(uint8_t *packet) {
//     if (packet == NULL) return;
//
//     // 1. 校验和计算
//     // SUM = 0x55 + 0x53 + RollL + RollH + ... + VH
//     uint8_t sum = 0;
//     for (int i = 0; i < 10; i++) {
//         sum += packet[i];
//     }
//
//     // 2. 校验通过则解析
//     if (sum == packet[10]) {
//         // 合并高低八位，注意先转为 int16_t 处理符号位
//         int16_t raw_roll  = (int16_t)((packet[3] << 8) | packet[2]);
//         int16_t raw_pitch = (int16_t)((packet[5] << 8) | packet[4]);
//         int16_t raw_yaw   = (int16_t)((packet[7] << 8) | packet[6]);
//         uint16_t raw_ver  = (uint16_t)((packet[9] << 8) | packet[8]);
//
//         // 公式转换: Angle = Raw / 32768 * 180
//         hwt_data.roll  = (float)raw_roll  / 32768.0f * 180.0f;
//         hwt_data.pitch = (float)raw_pitch / 32768.0f * 180.0f;
//         hwt_data.yaw   = (float)raw_yaw   / 32768.0f * 180.0f;
//
//         hwt_data.version = raw_ver;
//         hwt_data.update_count++;
//     }
// }
//
// /**
//  * @brief 串口回调函数
//  * 在DMA接收到数据或IDLE中断时被 bsp_usart 调用
//  */
// static void HWT606_Callback() {
//     uint8_t *buf = hwt_uart_instance->rx_buffer;
//     uint16_t len = RX_BUF_SIZE; // 或者是实际接收长度，但在你的bsp中是固定buffer检查
//
//     // 遍历Buffer寻找包头 0x55 0x53
//     // 注意：防止数组越界，i < len - HWT_DATA_LEN
//     for (int i = 0; i < len - HWT_DATA_LEN; i++) {
//         if (buf[i] == HWT_HEADER && buf[i+1] == HWT_TYPE_ANGLE) {
//             // 找到包头，传入当前位置进行处理
//             Process_Angle_Packet(&buf[i]);
//
//             // 优化：HWT606通常连续发送多个包，如果只需要角度，
//             // 处理完一个后可以跳过这11个字节继续找，或者直接return
//             // 这里选择跳过，以防一帧buffer里有多个数据包
//             i += (HWT_DATA_LEN - 1);
//         }
//     }
// }
//
// /**
//  * @brief 初始化并注册
//  */
// HWT606_Data_t* HWT606_Init(UART_HandleTypeDef *hwt_uart_handle) {
//     if (hwt_uart_handle == NULL) return NULL;
//
//     // 清零数据
//     memset(&hwt_data, 0, sizeof(HWT606_Data_t));
//
//     // 注册到你的 bsp_usart 系统
//     // 假设陀螺仪接在指定的UART上
//     hwt_uart_instance = Uart_register(hwt_uart_handle, HWT606_Callback);
//
//     return &hwt_data;
// }
//
// HWT606_Data_t* HWT606_GetData(void) {
//     return &hwt_data;
// }