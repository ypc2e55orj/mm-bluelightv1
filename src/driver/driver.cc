#include "driver.h"

// C++
#include <memory>

namespace driver {
Driver::Driver() {}
Driver::~Driver() = default;

void Driver::init_pro() {
  // clang-format off
  battery = std::make_unique<hardware::Battery>(
      ADC_UNIT_BATTERY,
      ADC_CHANNEL_BATTERY);

  spi_imu_ = std::make_unique<peripherals::Spi>(
      SPI3_HOST,
      GPIO_NUM_IMU_SPI_MISO,
      GPIO_NUM_IMU_SPI_MOSI,
      GPIO_NUM_IMU_SPI_SCLK,
      16);
  imu = std::make_unique<hardware::Imu>(
      *spi_imu_,
      GPIO_NUM_IMU_SPI_CS);

  spi_encoder_ = std::make_unique<peripherals::Spi>(
      SPI2_HOST,
      GPIO_NUM_ENCODER_SPI_MISO,
      GPIO_NUM_ENCODER_SPI_MOSI,
      GPIO_NUM_ENCODER_SPI_SCLK,
      4);
  encoder_left = std::make_unique<hardware::Encoder>(
      *spi_encoder_,
      GPIO_NUM_ENCODER_SPI_CS_LEFT);
  encoder_right = std::make_unique<hardware::Encoder>(
      *spi_encoder_,
      GPIO_NUM_ENCODER_SPI_CS_RIGHT);

  motor_left = std::make_unique<hardware::Motor>(
      0,
      GPIO_NUM_MOTOR_LEFT_IN1,
      GPIO_NUM_MOTOR_LEFT_IN2);
  motor_right = std::make_unique<hardware::Motor>(
      1,
      GPIO_NUM_MOTOR_RIGHT_IN1,
      GPIO_NUM_MOTOR_RIGHT_IN2);

  hardware::Photo::Config config{
      .adc_unit = ADC_UNIT_PHOTO,
      .adc_channel = {
          ADC_CHANNEL_PHOTO_LEFT90,
          ADC_CHANNEL_PHOTO_LEFT45,
          ADC_CHANNEL_PHOTO_RIGHT45,
          ADC_CHANNEL_PHOTO_RIGHT90},
      .gpio_num = {
          GPIO_NUM_PHOTO_LEFT90,
          GPIO_NUM_PHOTO_LEFT45,
          GPIO_NUM_PHOTO_RIGHT45,
          GPIO_NUM_PHOTO_RIGHT90}};
  photo = std::make_unique<hardware::Photo>(config);
  // clang-format off
}

void Driver::init_app() {
  // clang-format off
  uart = std::make_unique<hardware::Uart>();
  fs = std::make_unique<hardware::Fs>(10);

  indicator = std::make_unique<hardware::Indicator>(
      GPIO_NUM_INDICATOR,
      NUM_INDICATORS);

  buzzer = std::make_unique<hardware::Buzzer>(
      GPIO_NUM_BUZZER);
  // clang-format on
}
}  // namespace driver