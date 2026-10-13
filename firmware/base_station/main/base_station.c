#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 工程初始化验证用：证明 ESP32-S3 工程可编译/烧录/运行。
// 业务功能（SoftAP/UDP/CSI/SD/OLED）后续在此之上逐步添加。
void app_main(void)
{
    printf("base_station (ESP32-S3) project initialized.\n");
}
