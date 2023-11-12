#include "driver.h"

// C++
#include <memory>

namespace driver {
Driver::Driver() {
  spi_encoder_ = std::make_unique<peripherals::Spi>(
      SPI2_HOST, GPIO_NUM_ENCODER_SPI_MISO, GPIO_NUM_ENCODER_SPI_MOSI,
      GPIO_NUM_ENCODER_SPI_SCLK, 4);
  spi_imu_ = std::make_unique<peripherals::Spi>(
      SPI3_HOST, GPIO_NUM_IMU_SPI_MISO, GPIO_NUM_IMU_SPI_MOSI,
      GPIO_NUM_IMU_SPI_SCLK, 16);
  buzzer = std::make_unique<hardware::Buzzer>(GPIO_NUM_BUZZER);
  battery = std::make_unique<hardware::Battery>(ADC_UNIT_BATTERY,
                                                ADC_CHANNEL_BATTERY);
  imu = std::make_unique<hardware::Imu>(*spi_imu_, GPIO_NUM_IMU_SPI_CS);
  encoder_left = std::make_unique<hardware::Encoder>(
      *spi_encoder_, GPIO_NUM_ENCODER_SPI_CS_LEFT);
  encoder_right = std::make_unique<hardware::Encoder>(
      *spi_encoder_, GPIO_NUM_ENCODER_SPI_CS_RIGHT);
  fs = std::make_unique<hardware::Fs>(10);
  motor_left = std::make_unique<hardware::Motor>(0, GPIO_NUM_MOTOR_LEFT_IN1,
                                                 GPIO_NUM_MOTOR_LEFT_IN2);
  motor_right = std::make_unique<hardware::Motor>(1, GPIO_NUM_MOTOR_RIGHT_IN1,
                                                  GPIO_NUM_MOTOR_RIGHT_IN2);
  indicator =
      std::make_unique<hardware::Indicator>(GPIO_NUM_INDICATOR, NUM_INDICATORS);

  hardware::Photo::Config config{};
  config.adc_unit = ADC_UNIT_PHOTO;
  config.gpio_num[0] = GPIO_NUM_PHOTO_LEFT90,
  config.adc_channel[0] = ADC_CHANNEL_PHOTO_LEFT90;
  config.gpio_num[1] = GPIO_NUM_PHOTO_LEFT45,
  config.adc_channel[1] = ADC_CHANNEL_PHOTO_LEFT45;
  config.gpio_num[2] = GPIO_NUM_PHOTO_RIGHT45,
  config.adc_channel[2] = ADC_CHANNEL_PHOTO_RIGHT45;
  config.gpio_num[3] = GPIO_NUM_PHOTO_RIGHT90,
  config.adc_channel[3] = ADC_CHANNEL_PHOTO_RIGHT90;
  photo = std::make_unique<hardware::Photo>(config);
}
Driver::~Driver() = default;
}  // namespace driver