目的：支持一切使用CAN通信的设备，提供普适化的底层支持

每一个CAN设备都会包含一个can设备管理者，类型为Can_controller_t
Can_controller_t定义如下：
typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄，必须有
    CAN_TxHeaderTypeDef tx_config; // CAN报文发送配置，必须有
    uint32_t tx_mailbox;           // CAN消息填入的邮箱，不需要填
    uint32_t can_id;               // 发送所需要的CAN总线ID，必须有
    uint32_t tx_id;                // 设备ID，由设备闪烁次数或拨码开关决定，其值一般为1-8，可有可无
    uint8_t rx_buffer[8];          // 接收缓冲区
    uint32_t rx_id;                // 接收ID，必须有
    // 接收的回调函数指针,用于解析接收到的数据
    void (*receive_callback)(struct _*);//等效于参数传入为一个Can_controller_t类型的结构体指针
} Can_controller_t; 

每一个Can管理者在初始化的时候都要使用其对应初始化结构体，类型为Can_init_t
定义如下：
typedef struct can_init
{
CAN_HandleTypeDef *can_handle; // can句柄
CAN_TxHeaderTypeDef tx_config; // CAN报文发送配置
uint32_t can_id;               //电机ID，由电机的闪烁次数或拨码开关决定
uint32_t tx_id;                 
uint32_t rx_id;                // 接收ID,由电机类型以及can_id进行计算，也可以手动输入
void (*receive_callback)( Can_controller_t* );
} Can_init_t;

使用接口说明：
1.Can设备初始化：Can_controller_t *Can_device_init(Can_init_t *can_config);
步骤：
    //定义CAN设备初始化结构体
    Can_init_t *can_config;
    can_config = 
    {
        .can_handle = &hcan1,
        .can_id     = 0x1FF,
        .tx_id      = 2,
        .rx_id      = 0x206 
        .receive_callback = Decoder_djimotor,
    }
    //定义CAN设备结构体
    Can_controller_t *can_dev = Can_device_init(can_config);
2.Can发送信息
步骤：
    //使用已初始化的CAN设备进行发送，沿用上面定义的can_dev
    //每次发送都会发送8字节，因此你可以写好发送缓冲区
    uint8_t tx_0x1ff_buffer[8]={0x11,0x32}//只有前两个字节有用
    //调用发送函数
    Can_send_data(can_dev,tx_0x1ff_buffer);
    
