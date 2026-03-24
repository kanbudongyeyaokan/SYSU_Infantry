#include "vision_comm.h"
#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "error_handler.h"

extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

static uint8_t rx_fifo[VISION_RX_FIFO_SIZE];
static uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

static Vision_Ctrl_Data_t latest_vision_ctrl_data;
static uint32_t last_valid_time = 0; 
static Uart_instance_t *vision_uart = NULL; 

static void Vision_Rx_Callback(void);
static uint16_t Get_FIFO_Data_Len(void);
static void Read_FIFO_Data(uint8_t* dest, uint16_t len, uint16_t offset);

void Vision_Comm_Init(void) {
    memset(&latest_vision_ctrl_data, 0, sizeof(Vision_Ctrl_Data_t));
    
    // 注册串口 1 (根据你的代码维持不变)
    vision_uart = Uart_register(&huart6, Vision_Rx_Callback);
    
    if (vision_uart != NULL) {
        ERROR_INFO("VISION", "Init OK, UART1 registered");
    } else {
        ERROR_CRITICAL("VISION", "Init FAILED, UART1 register error");
    }
}

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

// 核心解析任务 
void Vision_Comm_Parse_Task(void) {
    uint8_t header_buf[4]; // 预览: sof(1) + len(2) + cmd_id(1) = 4字节
    uint8_t frame_buf[128]; 
    
    while (Get_FIFO_Data_Len() >= 4) { 
        
        Read_FIFO_Data(header_buf, 1, 0);
        if (header_buf[0] != VISION_SOF_RX) {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            continue;
        }
        
        Read_FIFO_Data(header_buf, 4, 0);
        uint16_t data_len = (header_buf[2] << 8) | header_buf[1]; 
        uint8_t cmd_id = header_buf[3];
        
        uint16_t frame_total_len = 4 + data_len + 2; 
        
        if (frame_total_len > 128) {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
            continue;
        }
        
        if (Get_FIFO_Data_Len() < frame_total_len) {
            break; 
        }
        
        Read_FIFO_Data(frame_buf, frame_total_len, 0);
        
        if (Verify_CRC16_Check_Sum(frame_buf, frame_total_len) == 1) {
            if (cmd_id == CMD_ID_CTRL_RX && data_len == sizeof(Vision_Rx_Payload_t)) {
                
                Vision_Rx_Payload_t *rx_payload = (Vision_Rx_Payload_t *)&frame_buf[4];
                
                taskENTER_CRITICAL();
                
                // 1. 组合状态机映射
                uint8_t is_tracking = (rx_payload->flags & 0x02) >> 1; // 取 bit 1
                uint8_t is_fire     = (rx_payload->flags & 0x04) >> 2; // 取 bit 2
                
                if (is_tracking && is_fire) {
                    latest_vision_ctrl_data.tracking_state = 2; // 锁死，允许射击
                } else if (is_tracking) {
                    latest_vision_ctrl_data.tracking_state = 1; // 仅追踪
                } else {
                    latest_vision_ctrl_data.tracking_state = 0; // 丢失
                }
                
                // 2. 映射控制量
                latest_vision_ctrl_data.target_pitch = rx_payload->angular_y;
                latest_vision_ctrl_data.target_yaw   = rx_payload->angular_z;
                
                // 新协议缺少速度前馈，强制置0，保证安全
                latest_vision_ctrl_data.target_pitch_v = 0.0f; 
                latest_vision_ctrl_data.target_yaw_v   = 0.0f;
                
                // 3. 映射底盘透传量
                latest_vision_ctrl_data.linear_x = rx_payload->linear_x;
                latest_vision_ctrl_data.linear_y = rx_payload->linear_y;
                latest_vision_ctrl_data.linear_z = rx_payload->linear_z;
                latest_vision_ctrl_data.angular_x = rx_payload->angular_x;

                last_valid_time = osKernelSysTick(); 
                taskEXIT_CRITICAL();
            }
            rx_tail = (rx_tail + frame_total_len) % VISION_RX_FIFO_SIZE;
        } else {
            rx_tail = (rx_tail + 1) % VISION_RX_FIFO_SIZE;
        }
    }
}

// 高频发送姿态包 
// 修改了函数签名，去掉了废弃的 roll，加入了血量参数，方便你在 NUC 端直接监控
void Vision_Send_Pose(uint32_t time_us, float pitch, float yaw, float pitch_v, float yaw_v, uint16_t current_hp, uint16_t max_hp) {
    if (vision_uart == NULL) return;
    
    EC2Vision_Pose_t tx_frame;
    
    // 【极其关键】必须将整个帧清零！否则 NUC 收到的 HP、进度等预留字段全是内存垃圾数据
    memset(&tx_frame, 0, sizeof(EC2Vision_Pose_t)); 
    
    // 1. 填充协议头
    tx_frame.sof = VISION_SOF_TX;
    tx_frame.data_length = sizeof(Vision_Tx_Payload_t); // 直接填入 Payload 的准确大小
    tx_frame.cmd_id = CMD_ID_POSE_TX;
    
    // 2. 填充核心位姿数据 (注意要带上 .data.)
    tx_frame.data.timestamp_us = time_us;
    tx_frame.data.angular_y = pitch;         // 对应 Python 中的 state.angular_y
    tx_frame.data.angular_z = yaw;           // 对应 Python 中的 state.angular_z
    tx_frame.data.angular_y_speed = pitch_v;
    tx_frame.data.angular_z_speed = yaw_v;
    
    // 3. 填充裁判系统血量数据 (供 NUC 端显示)
    tx_frame.data.current_HP = current_hp;
    tx_frame.data.maximum_HP = max_hp;
    
    // 4. 追加 CRC16 校验 
    Append_CRC16_Check_Sum((uint8_t*)&tx_frame, sizeof(EC2Vision_Pose_t) - 2);
    
    // 5. 调用底层非阻塞发送
    Uart_sendData(vision_uart, (uint8_t*)&tx_frame, sizeof(EC2Vision_Pose_t));
}

const Vision_Ctrl_Data_t* Get_Vision_Ctrl_Data(void) {
    return &latest_vision_ctrl_data;
}

bool Is_Vision_Online(void) {
    return (osKernelSysTick() - last_valid_time) < 100; 
}