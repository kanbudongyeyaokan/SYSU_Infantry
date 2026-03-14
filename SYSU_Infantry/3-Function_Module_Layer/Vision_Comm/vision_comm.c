#include "vision_comm.h"
// ========== USB 方式（测试完成后切换回来） ==========
// #include "bsp_usb.h"

// ========== 串口方式（测试用） ==========
#include "bsp_usart.h"
#include "usart.h"
#include "crc_referee.h"
#include "cmsis_os.h"
#include "string.h"
#include "error_handler.h"
// ==========================================
// 内部变量
// ==========================================
// 接收环形缓冲区
static uint8_t rx_fifo[VISION_RX_FIFO_SIZE];
static uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

// 存储最新解析出的有效数据
static Infantry_Vision_Rx_Data_t latest_vision_data;

// 记录最后一次收到有效数据的时间 (用于掉线检测)
static uint32_t last_valid_time = 0; 

// ========== 串口方式（测试用） ==========
static Uart_instance_t *vision_uart = NULL; 

// ==========================================
// 内部函数声明
// ==========================================
// ========== USB 方式 ==========
// static void Vision_Rx_Callback(uint8_t* buf, uint32_t len);

// ========== 串口方式（测试用） ==========
static void Vision_Rx_Callback(void);

static uint16_t Get_FIFO_Data_Len(void);
static void Read_FIFO_Data(uint8_t* dest, uint16_t len, uint16_t offset);

// ==========================================
// 接口实现
// ==========================================

void Vision_Comm_Init(void) {
    memset(&latest_vision_data, 0, sizeof(Infantry_Vision_Rx_Data_t));
    
    // ========== USB 方式（测试完成后切换回来） ==========
    // Usb_Init(Vision_Rx_Callback);
    
    // ========== 串口方式（测试用） ==========
    vision_uart = Uart_register(&huart6, Vision_Rx_Callback);
    
    if (vision_uart != NULL) {
        ERROR_INFO("VISION", "Init OK, UART6 registered");
    } else {
        ERROR_CRITICAL("VISION", "Init FAILED, UART6 register error");
    }
}

// ========== USB 方式 ==========
// USB 接收回调函数 (由 bsp_usb.c 在中断中触发)
// static void Vision_Rx_Callback(uint8_t* buf, uint32_t len) {
//     if (buf == NULL || len == 0) return;
//     
//     // 极速将收到的数据推入 FIFO，绝不阻塞！
//     for (uint32_t i = 0; i < len; i++) {
//         rx_fifo[rx_head] = buf[i];
//         rx_head++;
//         if (rx_head >= VISION_RX_FIFO_SIZE) {
//             rx_head = 0;
//         }
//     }
// }

// ========== 串口方式（测试用） ==========
static void Vision_Rx_Callback(void) {
    if (vision_uart == NULL) {
        ERROR_WARN("VISION", "Rx callback called but UART not registered");
        return;
    }
    
    uint8_t *buf = vision_uart->rx_buffer;
    uint32_t len = vision_uart->rx_data_len;
    
    if (buf == NULL || len == 0) return;
    
    for (uint32_t i = 0; i < len; i++) {
        rx_fifo[rx_head] = buf[i];
        rx_head++;
        if (rx_head >= VISION_RX_FIFO_SIZE) {
            rx_head = 0;
        }
    }
}

// 计算 FIFO 中现存的数据量
static uint16_t Get_FIFO_Data_Len(void) {
    if (rx_head >= rx_tail) {
        return rx_head - rx_tail;
    } else {
        return VISION_RX_FIFO_SIZE - rx_tail + rx_head;
    }
}

// 从 FIFO 中读取数据但不移动 tail 指针 (用于预览/校验)
static void Read_FIFO_Data(uint8_t* dest, uint16_t len, uint16_t offset) {
    uint16_t read_idx = (rx_tail + offset) % VISION_RX_FIFO_SIZE;
    for (uint16_t i = 0; i < len; i++) {
        dest[i] = rx_fifo[read_idx];
        read_idx++;
        if (read_idx >= VISION_RX_FIFO_SIZE) {
            read_idx = 0;
        }
    }
}

// 核心解析任务 (滑动窗口法，极度鲁棒)
void Vision_Comm_Parse_Task(void) {
    uint8_t header_buf[5];
    uint8_t frame_buf[128]; // 足够容纳一帧最大长度即可
    
    while (Get_FIFO_Data_Len() >= 5) { // 至少要够一个帧头的大小
        
        // 1. 寻找帧头 SOF
        Read_FIFO_Data(header_buf, 1, 0);
        if (header_buf[0] != VISION_SOF) {
            // 滑动窗口：如果不是帧头，丢弃一个字节，继续找
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            continue;
        }
        
        // 2. 预览完整帧头并校验 CRC8
        Read_FIFO_Data(header_buf, 5, 0);
        // 使用大疆官方的 CRC8 校验函数 (长度传 5)
        if (Verify_CRC8_Check_Sum(header_buf, 5) == 0) {
            // 帧头校验失败，说明是伪造的 SOF 或者错位了，丢弃 SOF 继续找
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            ERROR_WARN("VISION","CRC8 fail, discarding byte");
            continue;
        }
        
        // 3. 帧头合法，解析数据长度
        Vision_Frame_Header_t* p_header = (Vision_Frame_Header_t*)header_buf;
        uint16_t data_len = p_header->data_length;
        uint16_t frame_total_len = 5 + 2 + data_len + 2; // 帧头(5) + CMD_ID(2) + 数据段 + CRC16(2)
        
        // 保护机制：如果解析出的长度超大(例如错包)，直接丢弃帧头
        if (frame_total_len > 128) {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            ERROR_WARN("VISION","Frame too long (%d), discarding byte", frame_total_len);

            continue;
        }
        
        // 4. 检查 FIFO 中是否有完整的一帧数据
        if (Get_FIFO_Data_Len() < frame_total_len) {
            // 数据还没收全（可能 USB 刚传一半），退出 while 等待下一次处理
            break; 
        }
        
        // 5. 提取整帧数据并校验 CRC16
        Read_FIFO_Data(frame_buf, frame_total_len, 0);
        // 使用大疆官方的 CRC16 校验函数
        if (Verify_CRC16_Check_Sum(frame_buf, frame_total_len) == 1) {
            // ==========================================
            // 校验完全通过，提取 Payload！
            // ==========================================
            uint16_t cmd_id = (frame_buf[6] << 8) | frame_buf[5];
            
            // 匹配步兵指令 ID 和数据长度
            if (cmd_id == CMD_ID_INFANTRY && data_len == sizeof(Infantry_Vision_Rx_Data_t)) {
                // 进入临界区，防止读写冲突
                taskENTER_CRITICAL();
                memcpy(&latest_vision_data, &frame_buf[7], sizeof(Infantry_Vision_Rx_Data_t));
                last_valid_time = osKernelSysTick(); // 更新有效时间戳
                taskEXIT_CRITICAL();
            }
            
            // 成功解析一帧，滑动指针跳过这整帧数据
            rx_tail = (rx_tail + frame_total_len) % VISION_RX_FIFO_SIZE;
        } else {
            // CRC16 错误，说明数据中途损坏，仅丢弃 SOF 继续找
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            ERROR_WARN("VISION","CRC16 fail, discarding byte");
        }
    }
}

// 获取数据的 Getter，供云台控制调用
const Infantry_Vision_Rx_Data_t* Get_Vision_Data(void) {
    return &latest_vision_data;
}

// 掉线检测 (比如 500ms 没收到有效数据判定为离线)
bool Is_Vision_Online(void) {
    return (osKernelSysTick() - last_valid_time) < 500; 
}