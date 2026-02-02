#include "buzzer_alarm.h"
#include "buzzer_alarm_task.h"
#include "queue.h"
#include "robot_task.h"


#define BUZZER_ALARM_TASK_PERIOD 100

void Buzzer_alarm_control_task(void const *argument) {
    // 初始化
    Buzzer_alarm_init();

    Buzzer_Alarm_Type_e type;
    // 绝对延时变量
    TickType_t PreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        if(xQueueReceive(Buzzer_cmd_queue_handle, &type, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Buzzer_alarm_handle_command(type);

            type = BUZZER_ALARM_NONE;
        }

        vTaskDelayUntil(&PreviousWakeTime, BUZZER_ALARM_TASK_PERIOD);


    }
}