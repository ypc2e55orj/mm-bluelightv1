#pragma once

// C++
#include <memory>

// ESP-IDF

// Project
#include "hardware/battery.hpp"
#include "hardware/buzzer.hpp"
#include "hardware/encoder.hpp"
#include "hardware/fs.hpp"
#include "hardware/imu.hpp"
#include "hardware/indicator.hpp"
#include "hardware/motor.hpp"
#include "hardware/photo.hpp"
#include "peripherals/spi.hpp"

namespace driver {
// Encoder
constexpr auto GPIO_NUM_ENCODER_SPI_MISO = GPIO_NUM_37;
constexpr auto GPIO_NUM_ENCODER_SPI_MOSI = GPIO_NUM_35;
constexpr auto GPIO_NUM_ENCODER_SPI_SCLK = GPIO_NUM_36;
constexpr auto GPIO_NUM_ENCODER_SPI_CS_LEFT = GPIO_NUM_26;
constexpr auto GPIO_NUM_ENCODER_SPI_CS_RIGHT = GPIO_NUM_39;

// Imu
constexpr auto GPIO_NUM_IMU_SPI_MISO = GPIO_NUM_48;
constexpr auto GPIO_NUM_IMU_SPI_MOSI = GPIO_NUM_47;
constexpr auto GPIO_NUM_IMU_SPI_SCLK = GPIO_NUM_33;
constexpr auto GPIO_NUM_IMU_SPI_CS = GPIO_NUM_34;

// Buzzer
constexpr auto GPIO_NUM_BUZZER = GPIO_NUM_21;

// Indicator
constexpr auto GPIO_NUM_INDICATOR = GPIO_NUM_45;
constexpr auto NUM_INDICATORS = 4;

// Motor
constexpr auto GPIO_NUM_MOTOR_LEFT_IN1 = GPIO_NUM_40;
constexpr auto GPIO_NUM_MOTOR_LEFT_IN2 = GPIO_NUM_38;
constexpr auto GPIO_NUM_MOTOR_RIGHT_IN1 = GPIO_NUM_42;
constexpr auto GPIO_NUM_MOTOR_RIGHT_IN2 = GPIO_NUM_41;

// Photo
constexpr auto GPIO_NUM_PHOTO_LEFT90 = GPIO_NUM_13;
constexpr auto GPIO_NUM_PHOTO_LEFT45 = GPIO_NUM_12;
constexpr auto GPIO_NUM_PHOTO_RIGHT45 = GPIO_NUM_11;
constexpr auto GPIO_NUM_PHOTO_RIGHT90 = GPIO_NUM_10;
constexpr auto ADC_UNIT_PHOTO = ADC_UNIT_1;
constexpr auto ADC_CHANNEL_PHOTO_LEFT90 = ADC_CHANNEL_3;
constexpr auto ADC_CHANNEL_PHOTO_LEFT45 = ADC_CHANNEL_2;
constexpr auto ADC_CHANNEL_PHOTO_RIGHT45 = ADC_CHANNEL_1;
constexpr auto ADC_CHANNEL_PHOTO_RIGHT90 = ADC_CHANNEL_0;

// Battery
constexpr auto ADC_UNIT_BATTERY = ADC_UNIT_1;
constexpr auto ADC_CHANNEL_BATTERY = ADC_CHANNEL_4;

class Driver {
 private:
  std::unique_ptr<peripherals::Spi> spi_encoder_;
  std::unique_ptr<peripherals::Spi> spi_imu_;

 public:
  std::unique_ptr<hardware::Battery> battery;
  std::unique_ptr<hardware::Buzzer> buzzer;
  std::unique_ptr<hardware::Encoder> encoder_left;
  std::unique_ptr<hardware::Encoder> encoder_right;
  std::unique_ptr<hardware::Fs> fs;
  std::unique_ptr<hardware::Imu> imu;
  std::unique_ptr<hardware::Indicator> indicator;
  std::unique_ptr<hardware::Motor> motor_left;
  std::unique_ptr<hardware::Motor> motor_right;
  std::unique_ptr<hardware::Photo> photo;

  explicit Driver();
  ~Driver();
};
}  // namespace driver
