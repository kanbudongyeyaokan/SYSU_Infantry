/**
 * @file    jy61p_iic.c
 * @brief   JY61P I2C 驱动实现
 *
 * 协议要点：
 * - I2C 默认 7-bit 地址：0x50
 * - 关键寄存器（每个寄存器是 16-bit，小端）：
 *   AX~AZ: 0x34~0x36
 *   GX~GZ: 0x37~0x39
 *   Roll~Yaw: 0x3D~0x3F
 *   Temp: 0x40
 */

#include "jy61p_iic.h"

#include <string.h>

#include "bsp_wdg.h"
#include "buzzer_alarm.h"
#include "error_handler.h"

#define JY61P_REG_ACC_START      0x34u
#define JY61P_REG_TEMP           0x40u
#define JY61P_READ_LEN           26u
#define JY61P_CALIB_SAMPLES      100u
#define JY61P_I2C_TIMEOUT_MS     5u
#define JY61P_READY_RETRY_MAX    5u
#define JY61P_GRAVITY            9.80665f

typedef struct {
    I2C_HandleTypeDef *hi2c;   /* 绑定的 I2C 外设 */
    uint8_t dev_addr;          /* 器件地址（左移后的 HAL 地址） */

    uint8_t frame_buf[JY61P_READ_LEN]; /* 连续读取缓存 */

    bool is_ready;             /* 初始化成功标记 */
    bool read_success;         /* 本轮读取是否成功 */

    bool is_calibrated;        /* 是否完成 yaw 零偏校准 */
    uint16_t calib_cnt;        /* 校准累计样本数 */
    float yaw_sum;             /* 校准阶段 yaw 累计值 */
    float yaw_offset;          /* 计算得到的 yaw 零偏 */
    float last_yaw;            /* 上一帧 yaw，用于多圈统计 */
} JY61P_Driver_t;

static JY61P_Driver_t jy61p_dev;
static Watchdog_device_t *jy61p_wdg = NULL;

/* 按寄存器号读取 16 位有符号数据（低字节在前）。 */
static int16_t JY61P_ReadInt16ByReg(const uint8_t *buf, uint8_t reg)
{
    const uint16_t idx = (uint16_t)(reg - JY61P_REG_ACC_START) * 2u;
    return (int16_t)(((uint16_t)buf[idx + 1u] << 8u) | buf[idx]);
}

static float JY61P_Normalize180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void JY61P_Offline_Callback(void *arg)
{
    (void)arg;
    /* 掉线后复用现有蜂鸣器报警机制。 */
    Watchdog_buzzer_alarm("imu");
}

static bool JY61P_Init(void)
{
    uint8_t retry = 0u;

    if (jy61p_dev.hi2c == NULL) {
        ERROR_CRITICAL("JY61P", "I2C handle is NULL");
        return false;
    }

    memset(jy61p_dev.frame_buf, 0, sizeof(jy61p_dev.frame_buf));
    jy61p_dev.read_success = false;
    jy61p_dev.is_calibrated = false;
    jy61p_dev.calib_cnt = 0u;
    jy61p_dev.yaw_sum = 0.0f;
    jy61p_dev.yaw_offset = 0.0f;
    jy61p_dev.last_yaw = 0.0f;

    /* 上电后留一点稳定时间，再进行在线检测。 */
    HAL_Delay(20u);
    while (HAL_I2C_IsDeviceReady(jy61p_dev.hi2c, jy61p_dev.dev_addr, 3u, 10u) != HAL_OK) {
        retry++;
        if (retry >= JY61P_READY_RETRY_MAX) {
            ERROR_CRITICAL("JY61P", "device not ready addr=0x%02X", jy61p_dev.dev_addr);
            return false;
        }
        HAL_Delay(10u);
    }

    {
        Watchdog_init_t wdg_config = {
            .owner_id = NULL,
            .reload_count = 30u,
            .callback = JY61P_Offline_Callback,
            .name = "imu"
        };
        jy61p_wdg = Watchdog_register(&wdg_config);
    }

    jy61p_dev.is_ready = true;
    return true;
}

static void JY61P_Start_Read(void)
{
    if (!jy61p_dev.is_ready) {
        return;
    }

    jy61p_dev.read_success = false;

    if (HAL_I2C_Mem_Read(jy61p_dev.hi2c,
                         jy61p_dev.dev_addr,
                         JY61P_REG_ACC_START,
                         I2C_MEMADD_SIZE_8BIT,
                         jy61p_dev.frame_buf,
                         JY61P_READ_LEN,
                         JY61P_I2C_TIMEOUT_MS) == HAL_OK) {
        /* start_read 阶段已完成实际读取，wait_data 仅做结果确认。 */
        jy61p_dev.read_success = true;
    }
}

static bool JY61P_Wait_Data(void)
{
    if (jy61p_dev.read_success && (jy61p_wdg != NULL)) {
        Watchdog_feed(jy61p_wdg);
    }
    return jy61p_dev.read_success;
}

static void JY61P_Process(Ins_data_t *out_data, float dt_s)
{
    const float k_acc = 16.0f / 32768.0f * JY61P_GRAVITY;
    const float k_gyro = 2000.0f / 32768.0f;
    const float k_angle = 180.0f / 32768.0f;
    const float k_temp = 0.01f;

    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;
    int16_t temp_raw;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float final_yaw_deg;

    (void)dt_s;

    if (!jy61p_dev.read_success) {
        return;
    }

    ax_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x34u);
    ay_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x35u);
    az_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x36u);
    gx_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x37u);
    gy_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x38u);
    gz_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x39u);
    roll_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x3Du);
    pitch_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x3Eu);
    yaw_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, 0x3Fu);
    temp_raw = JY61P_ReadInt16ByReg(jy61p_dev.frame_buf, JY61P_REG_TEMP);

    /*
     * 坐标映射策略与现有 HWT606 保持一致：
     * body_x <- sensor_y, body_y <- sensor_x, body_z <- sensor_z
     */
    roll_deg = (float)pitch_raw * k_angle;
    pitch_deg = (float)roll_raw * k_angle;
    yaw_deg = (float)yaw_raw * k_angle;

    if (!jy61p_dev.is_calibrated) {
        /* 启动阶段仅做 yaw 软件零偏校准，避免初始航向漂移。 */
        jy61p_dev.yaw_sum += yaw_deg;
        jy61p_dev.calib_cnt++;

        if (jy61p_dev.calib_cnt >= JY61P_CALIB_SAMPLES) {
            jy61p_dev.yaw_offset = jy61p_dev.yaw_sum / (float)jy61p_dev.calib_cnt;
            jy61p_dev.is_calibrated = true;
            jy61p_dev.last_yaw = JY61P_Normalize180(yaw_deg - jy61p_dev.yaw_offset);
            out_data->round_count = 0;
        }

        out_data->state = INS_STATE_CALIBRATING;
        out_data->euler.roll = 0.0f;
        out_data->euler.pitch = 0.0f;
        out_data->euler.yaw = 0.0f;
        out_data->gyro_body.x = 0.0f;
        out_data->gyro_body.y = 0.0f;
        out_data->gyro_body.z = 0.0f;
        out_data->acc_body.x = 0.0f;
        out_data->acc_body.y = 0.0f;
        out_data->acc_body.z = 0.0f;
        out_data->total_yaw = 0.0f;
        out_data->temp = 0.0f;
        return;
    }

    final_yaw_deg = JY61P_Normalize180(yaw_deg - jy61p_dev.yaw_offset);
    {
        /* 基于跨越 -180/180 的跳变做多圈计数。 */
        const float yaw_diff = final_yaw_deg - jy61p_dev.last_yaw;
        if (yaw_diff < -180.0f) {
            out_data->round_count++;
        } else if (yaw_diff > 180.0f) {
            out_data->round_count--;
        }
    }
    jy61p_dev.last_yaw = final_yaw_deg;

    out_data->euler.roll = JY61P_Normalize180(roll_deg);
    out_data->euler.pitch = JY61P_Normalize180(pitch_deg);
    out_data->euler.yaw = final_yaw_deg;

    out_data->gyro_body.x = (float)gy_raw * k_gyro;
    out_data->gyro_body.y = (float)gx_raw * k_gyro;
    out_data->gyro_body.z = (float)gz_raw * k_gyro;

    out_data->acc_body.x = (float)ay_raw * k_acc;
    out_data->acc_body.y = (float)ax_raw * k_acc;
    out_data->acc_body.z = (float)az_raw * k_acc;

    out_data->temp = (float)temp_raw * k_temp;
    out_data->total_yaw = (float)out_data->round_count * 360.0f + final_yaw_deg;
    out_data->state = INS_STATE_READY;
}

static const Ins_driver_interface_t jy61p_iic_drv = {
    .init = JY61P_Init,
    .start_read = JY61P_Start_Read,
    .wait_data = JY61P_Wait_Data,
    .process_data = JY61P_Process
};

const Ins_driver_interface_t *JY61P_IIC_Get_Driver(I2C_HandleTypeDef *i2c_handle)
{
    jy61p_dev.hi2c = i2c_handle;
    jy61p_dev.dev_addr = JY61P_IIC_ADDR;
    jy61p_dev.is_ready = false;
    return &jy61p_iic_drv;
}
