/**
 * @file    ui_task.c
 * @brief   UI task source
 */

#include "ui_task.h"

#include "ui_default.h"
#include "ui_interface.h"

void Ui_task(void const *argument)
{
    (void)argument;

    ui_interface_init();
    ui_init_default();

    for (;;)
    {
        osDelay(100);
        ui_update_default();
    }
}
