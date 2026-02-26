#include "buzzer_alarm.h"
#include "buzzer_alarm_task.h"
#include "queue.h"
#include "robot_task.h"
#include "buzzer_music.h"

void Buzzer_alarm_control_task(void const *argument) {
    Buzzer_alarm_init();
    uint8_t recv_alarm_times = 0;

    for (;;)
    {
        BaseType_t xQueueStatus = xQueueReceive(Buzzer_cmd_queue_handle, &recv_alarm_times, portMAX_DELAY);

        if (xQueueStatus == pdPASS) {
            Alarm_handle_command(&recv_alarm_times);
        }
    }
    osDelay(10);
}