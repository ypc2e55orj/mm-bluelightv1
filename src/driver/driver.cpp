#include "driver.hpp"

namespace driver
{
  Context ctx;

  void Context::initialize()
  {
    spi2_ = new driver::peripherals::Spi(SPI2_HOST, GPIO_NUM_37, GPIO_NUM_35, GPIO_NUM_36, 4);
    spi3_ = new driver::peripherals::Spi(SPI3_HOST, GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_33, 16);

    buzzer = new driver::hardware::Buzzer(GPIO_NUM_21);
    battery = new driver::hardware::Battery(ADC_UNIT_1, ADC_CHANNEL_4);
    imu = new driver::hardware::Imu(*spi3_, GPIO_NUM_34);
    encoder_left = new driver::hardware::Encoder(*spi2_, GPIO_NUM_26);
    encoder_right = new driver::hardware::Encoder(*spi2_, GPIO_NUM_39);
    motor_left = new driver::hardware::Motor(0, GPIO_NUM_40, GPIO_NUM_38);
    motor_right = new driver::hardware::Motor(1, GPIO_NUM_42, GPIO_NUM_41);
    indicator = new driver::hardware::Indicator(GPIO_NUM_45, 4);

    driver::hardware::Photo::Config config{};
    config.adc_unit = ADC_UNIT_1;
    config.gpio_num[0] = GPIO_NUM_13, config.adc_channel[0] = ADC_CHANNEL_3;
    config.gpio_num[1] = GPIO_NUM_12, config.adc_channel[1] = ADC_CHANNEL_2;
    config.gpio_num[2] = GPIO_NUM_11, config.adc_channel[2] = ADC_CHANNEL_1;
    config.gpio_num[3] = GPIO_NUM_10, config.adc_channel[3] = ADC_CHANNEL_0;
    photo = new driver::hardware::Photo(config);
  }
}