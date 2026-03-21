#include "vision_comm.h"
#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "error_handler.h"

// 引入大疆官方的 CRC 计算库
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

// ==========================================
// 内部变量
// ==========================================
static uint8_t rx_fifo[VISION_RX_FIFO_SIZE];
static uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

// 存储最新解析出的视觉控制指令
static Vision_Ctrl_Data_t latest_vision_ctrl_data;
static uint32_t last_valid_time = 0; 

// 串口实例
static Uart_instance_t *vision_uart = NULL; 

// ==========================================
// 内部函数声明
// ==========================================
static void Vision_Rx_Callback(void);
static uint16_t Get_FIFO_Data_Len(void);
static void Read_FIFO_Data(uint8_t* dest, uint16_t len, uint16_t offset);

// ==========================================
// 接口实现
// ==========================================

void Vision_Comm_Init(void) {
    memset(&latest_vision_ctrl_data, 0, sizeof(Vision_Ctrl_Data_t));
    
    // 注册串口 6，绑定底层的空闲中断/DMA 回调
    vision_uart = Uart_register(&huart1, Vision_Rx_Callback);
    
    if (vision_uart != NULL) {
        ERROR_INFO("VISION", "Init OK, UART6 registered");
    } else {
        ERROR_CRITICAL("VISION", "Init FAILED, UART6 register error");
    }
}

// 串口接收回调 (将数据推入 FIFO)
static void Vision_Rx_Callback(void) {
    if (vision_uart == NULL) return;
    
    uint8_t *buf = vision_uart->rx_buffer;
    uint32_t len = vision_uart->rx_data_len;
    
    if (buf == NULL || len == 0) return;
    
    for (uint32_t i = 0; i < len; i++) {
        rx_fifo[rx_head] = buf[i];
        rx_head++;
        if (rx_head >= VISION_RX_FIFO_SIZE) rx_head = 0;
    }
}

static uint16_t Get_FIFO_Data_Len(void) {
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    else return VISION_RX_FIFO_SIZE - rx_tail + rx_head;
}

static void Read_FIFO_Data(uint8_t* dest, uint16_t len, uint16_t offset) {
    uint16_t read_idx = (rx_tail + offset) % VISION_RX_FIFO_SIZE;
    for (uint16_t i = 0; i < len; i++) {
        dest[i] = rx_fifo[read_idx];
        read_idx++;
        if (read_idx >= VISION_RX_FIFO_SIZE) read_idx = 0;
    }
}

// 核心解析任务 (滑动窗口法)
void Vision_Comm_Parse_Task(void) {
    uint8_t header_buf[4]; // 预览: sof(1) + len(2) + cmd_id(1) = 4字节
    uint8_t frame_buf[128]; 
    
    while (Get_FIFO_Data_Len() >= 4) { 
        
        // 寻找接收帧头 SOF (0x5A)
        Read_FIFO_Data(header_buf, 1, 0);
        if (header_buf[0] != VISION_SOF_RX) {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            continue;
        }
        
        // 解析长度
        Read_FIFO_Data(header_buf, 4, 0);
        uint16_t data_len = (header_buf[2] << 8) | header_buf[1]; // 小端模式解析
        uint8_t cmd_id = header_buf[3];
        
        uint16_t frame_total_len = 4 + data_len + 2; // 头(4) + 数据段 + CRC16(2)
        
        // 保护机制：过滤超长错包
        if (frame_total_len > 128) {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            continue;
        }
        
        // 等待整帧数据接收完毕
        if (Get_FIFO_Data_Len() < frame_total_len) {
            break; 
        }
        
        // 提取整帧数据并校验 CRC16
        Read_FIFO_Data(frame_buf, frame_total_len, 0);
        
        if (Verify_CRC16_Check_Sum(frame_buf, frame_total_len) == 1) {
            // 校验通过，匹配控制指令
            if (cmd_id == CMD_ID_CTRL_RX && data_len == sizeof(Vision_Ctrl_Data_t)) {
                
                // 进入临界区，安全拷贝最新数据
                taskENTER_CRITICAL();
                memcpy(&latest_vision_ctrl_data, &frame_buf[4], sizeof(Vision_Ctrl_Data_t));
                last_valid_time = osKernelSysTick(); // 更新心跳包时间
                taskEXIT_CRITICAL();
            }
            
            // 成功解析一帧，滑动跳过这整帧数据
            rx_tail = (rx_tail + frame_total_len) % VISION_RX_FIFO_SIZE;
        } else {
            // CRC16 错误，仅丢弃 SOF 继续找
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
        }
    }
}

// 高频发送姿态包 
void Vision_Send_Pose(uint32_t time_us, float pitch, float yaw, float roll, float pitch_v, float yaw_v) {
    if (vision_uart == NULL) return;
    
    EC2Vision_Pose_t tx_frame;
    
    // 填充协议头
    tx_frame.sof = VISION_SOF_TX;
    tx_frame.data_length = sizeof(EC2Vision_Pose_t) - 6; // 减去 sof, len, cmd_id, crc16
    tx_frame.cmd_id = CMD_ID_POSE_TX;
    
    // 填充核心数据
    tx_frame.timestamp_us = time_us;
    tx_frame.pitch_angle  = pitch;
    tx_frame.yaw_angle    = yaw;
    tx_frame.roll_angle   = roll;
    tx_frame.pitch_speed  = pitch_v;
    tx_frame.yaw_speed    = yaw_v;
    
    // 追加 CRC16 校验 
    Append_CRC16_Check_Sum((uint8_t*)&tx_frame, sizeof(EC2Vision_Pose_t));
    
    // 调用底层非阻塞发送
    Uart_sendData(vision_uart, (uint8_t*)&tx_frame, sizeof(EC2Vision_Pose_t));
}

const Vision_Ctrl_Data_t* Get_Vision_Ctrl_Data(void) {
    return &latest_vision_ctrl_data;
}

bool Is_Vision_Online(void) {
    // 100ms 没收到控制包，判定为掉线
    return (osKernelSysTick() - last_valid_time) < 100; 
}