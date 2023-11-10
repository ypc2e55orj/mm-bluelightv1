// C++
#include <cstdio>
#include <iostream>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "driver/hardware/battery.hpp"
#include "driver/hardware/buzzer.hpp"
#include "driver/hardware/encoder.hpp"
#include "driver/hardware/imu.hpp"
#include "driver/hardware/indicator.hpp"
#include "driver/hardware/motor.hpp"
#include "driver/hardware/photo.hpp"
#include "driver/peripherals/spi.hpp"

[[noreturn]] void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;
  driver::peripherals::Spi spi2(SPI2_HOST, GPIO_NUM_37, GPIO_NUM_35, GPIO_NUM_36, 4);
  driver::peripherals::Spi spi3(SPI3_HOST, GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_33, 16);

  driver::hardware::Buzzer buzzer(GPIO_NUM_21);
  driver::hardware::Battery battery(ADC_UNIT_1, ADC_CHANNEL_4);
  driver::hardware::Imu imu(spi3, GPIO_NUM_34);
  driver::hardware::Encoder encoder_left(spi2, GPIO_NUM_26);
  driver::hardware::Encoder encoder_right(spi2, GPIO_NUM_39);
  driver::hardware::Motor motor_left(0, GPIO_NUM_40, GPIO_NUM_38);
  driver::hardware::Motor motor_right(1, GPIO_NUM_42, GPIO_NUM_41);
  driver::hardware::Indicator indicator(GPIO_NUM_45, 4);

  driver::hardware::Photo::Config config{};
  config.adc_unit = ADC_UNIT_1;
  config.gpio_num[0] = GPIO_NUM_13, config.adc_channel[0] = ADC_CHANNEL_3;
  config.gpio_num[1] = GPIO_NUM_12, config.adc_channel[1] = ADC_CHANNEL_2;
  config.gpio_num[2] = GPIO_NUM_11, config.adc_channel[2] = ADC_CHANNEL_1;
  config.gpio_num[3] = GPIO_NUM_10, config.adc_channel[3] = ADC_CHANNEL_0;
  driver::hardware::Photo photo(config);

  printf("indicator & buzzer test\n");
  buzzer.start(8192, 10, 1);
  for (int i = 0; i < 100; i++)
  {
    buzzer.set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
    indicator.rainbow_yield();
    indicator.update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  indicator.clear();
  indicator.update();
  buzzer.stop();

  printf("motor test\n");
  motor_left.enable(), motor_right.enable();
  motor_left.speed(2000, 4000), motor_right.speed(2000, 4000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  motor_left.speed(0, 4000), motor_right.speed(0, 4000);
  motor_left.disable(), motor_right.disable();

  printf("\x1b[2J");
  printf("\x1b[?25l");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    photo.update();
    battery.update();
    imu.update();
    encoder_left.update();
    encoder_right.update();
    auto gyro = imu.gyro();
    auto accel = imu.accel();
    printf("\x1b[0;0H");
    printf("Battery\n"
           "  voltage: %4d\n"
           "  average: %4d\n\n",
           battery.voltage(), battery.average());

    printf("Photo\n"
           "  left90 : %4d, %4d\n"
           "  left45 : %4d, %4d\n"
           "  right45: %4d, %4d\n"
           "  right90: %4d, %4d\n\n",
           photo.left90().ambient, photo.left90().flash, photo.left45().ambient, photo.left45().flash,
           photo.right45().ambient, photo.right45().flash, photo.right90().ambient, photo.right90().flash);

    printf("Gyro \n"
           "  x: %f\n"
           "  y: %f\n"
           "  z: %f\n\n",
           gyro.x, gyro.y, gyro.z);

    printf("Accel\n"
           "  x: %f\n"
           "  y: %f\n"
           "  z: %f\n\n",
           accel.x, accel.y, accel.z);

    printf("Encoder\n"
           "  left : %f\n"
           "  right: %f\n\n",
           encoder_left.degree(), encoder_right.degree());
  }
}

// entrypoint
extern "C" [[maybe_unused]] void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr, 1);
}
