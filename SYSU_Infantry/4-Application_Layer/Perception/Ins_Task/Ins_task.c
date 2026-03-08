#include "Ins_task.h"

#include "bmi088.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "hwt606_iic.h"
#include "i2c.h"
#include "ins.h"
#include "jy61p_iic.h"

void Ins_task(void const *argument)
{
    extern I2C_HandleTypeDef hi2c2;
    extern UART_HandleTypeDef huart1;

    Uart_instance_t *imu_debug_uart = NULL;
    const Ins_driver_interface_t *driver = NULL;
    const Ins_data_t *data = NULL;
    uint32_t last_print_tick = 0u;

    (void)argument;

    /* IMU驱动切换区：当前使用 JY61P IIC。 */
    // driver = BMI088_Get_Driver();
    // driver = HWT606_IIC_Get_Driver(&hi2c2);
    driver = JY61P_IIC_Get_Driver(&hi2c2);

    Ins_init(driver);

    /* 额外注册 UART1，仅用于 JY61P 数据打印。 */
    imu_debug_uart = Uart_register(&huart1, NULL);

    for (;;) {
        Ins_update();
        data = Ins_get_data();

        /* 控制打印频率，避免串口刷屏过快。 */
        if ((imu_debug_uart != NULL) && ((HAL_GetTick() - last_print_tick) >= 50u)) {
            last_print_tick = HAL_GetTick();
            Uart_printf(
                imu_debug_uart,
                "JY61P st:%d r:%.2f p:%.2f y:%.2f gz:%.2f az:%.2f t:%.2f\r\n",
                (int)data->state,
                data->euler.roll,
                data->euler.pitch,
                data->euler.yaw,
                data->gyro_body.z,
                data->acc_body.z,
                data->temp
            );
        }

        osDelay(2);
    }
}

