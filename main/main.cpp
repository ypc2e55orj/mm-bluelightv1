#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <iostream>

#include "data/average.hpp"
#include "driver/hardware/battery.hpp"
#include "driver/hardware/buzzer.hpp"
#include "driver/hardware/encoder.hpp"
#include "driver/hardware/imu.hpp"
#include "driver/hardware/indicator.hpp"
#include "driver/hardware/photo.hpp"
#include "driver/peripherals/spi.hpp"

void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;
  /*
  driver::peripherals::Spi spi2(SPI2_HOST, GPIO_NUM_37, GPIO_NUM_35, GPIO_NUM_36, 4);
  driver::peripherals::Spi spi3(SPI3_HOST, GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_33, 16);

  driver::hardware::Buzzer buzzer(GPIO_NUM_21);
  driver::hardware::Battery battery(ADC_UNIT_1, ADC_CHANNEL_4);
  driver::hardware::Imu imu(spi3, GPIO_NUM_34);
  driver::hardware::Encoder left(spi2, GPIO_NUM_26);
  driver::hardware::Encoder right(spi2, GPIO_NUM_39);
  */
  driver::hardware::Indicator indicator(GPIO_NUM_45, 4);
  indicator.clear();
  indicator.update();

  driver::hardware::Photo::Config config;
  config.adc_unit = ADC_UNIT_1;
  config.gpio_num[0] = GPIO_NUM_13, config.adc_channel[0] = ADC_CHANNEL_3;
  config.gpio_num[1] = GPIO_NUM_12, config.adc_channel[1] = ADC_CHANNEL_2;
  config.gpio_num[2] = GPIO_NUM_11, config.adc_channel[2] = ADC_CHANNEL_1;
  config.gpio_num[3] = GPIO_NUM_10, config.adc_channel[3] = ADC_CHANNEL_0;
  driver::hardware::Photo photo(config);

  /*
  buzzer.start(8192, 10, 1);
  buzzer.set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
  */

  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    photo.update();
    printf("left90: %d, %d\n", photo.left90().ambient, photo.left90().flash);
    printf("left45: %d, %d\n", photo.left45().ambient, photo.left45().flash);
    printf("right45: %d, %d\n", photo.right45().ambient, photo.right45().flash);
    printf("right90: %d, %d\n", photo.right90().ambient, photo.right90().flash);
    /*
    imu.update();
    left.update();
    right.update();
    auto gyro = imu.gyro();
    auto accel = imu.accel();
    printf("\x1b[0;0H");
    printf("Gyro \nx: %f\ny: %f\nz: %f\n", gyro.x, gyro.y, gyro.z);
    printf("Accel\nx: %f\ny: %f\nz: %f\n", accel.x, accel.y, accel.z);
    printf("Encoder\nleft : %f\nright: %f\n", left.degree(), right.degree());
    */
  }
}

// entrypoint
extern "C" void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr, 1);
}
