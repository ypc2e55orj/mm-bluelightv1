#include "misc.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void wait_ms(uint32_t wtime)		//mS単位で待ち時間を生成する
{
  auto xLastWakeTime = xTaskGetTickCount();

	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(wtime));
}
