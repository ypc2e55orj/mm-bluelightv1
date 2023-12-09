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

void ANIMATE(void)
{
	BEEP();
  for (int i = 0; i < driver::indicator::nums(); i++)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(50));
    driver::indicator::clear();
  }
  for (int i = driver::indicator::nums() - 1; i > -1; i--)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(50));
    driver::indicator::clear();
  }
}

void BEEP(void){
	//driver::buzzer::tone(4000, 100);
	vTaskDelay(pdMS_TO_TICKS(100));
}

void BEEP_BUSY() {
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
		ANIMATE();
		LED(*mode);
	}

	if(speed_r < -0.1){
		if(mode == 0){
			*mode = 15;
		}else{
			*mode = *mode -1;
		}
		ANIMATE();
		LED(*mode);
	}
}
