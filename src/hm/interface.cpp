#include "math.h"
#include "sci.h"
#include "parameters.h"
#include "glob_var.h"
#include "interface.h"

#include "../driver/indicator.h"
#include "../driver/buzzer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void LED(short led_num){
	short bit = 1;

	driver::indicator::clear();
	for (int i = 0; i < driver::indicator::nums(); i++)
	{
		if (led_num & (bit << i))
		{
			driver::indicator::set(i , 0x00000F);
		}
	}
}

void BEEP_BUSY(void){
	driver::buzzer::tone(4000, 100);
	vTaskDelay(pdMS_TO_TICKS(100));
}

void BEEP() {
	driver::buzzer::tone(4000, 50);
	vTaskDelay(pdMS_TO_TICKS(100));
}

void mode_change( char* mode){
	if(speed_r > 0.1){
		if(*mode == 15){
			*mode = 0;
		}else{
			*mode= *mode + 1;
		}
		BEEP();
		LED(*mode);
	}

	if(speed_r < -0.1){
		if(mode == 0){
			*mode = 15;
		}else{
			*mode = *mode -1;
		}
		BEEP();
		LED(*mode);
	}
}
