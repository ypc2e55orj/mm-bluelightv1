#include "motor.h"

#include <driver/gpio.h>
#include <bdc_motor.h>

#include <cmath>

namespace driver::motor
{
  static const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 80'000'000; // 80MHz
  static const uint32_t BDC_MCPWM_FREQ_HZ = 100'000;                // 100kHz
  static constexpr uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ;

  static const gpio_num_t AIN1 = GPIO_NUM_42;
  static const gpio_num_t AIN2 = GPIO_NUM_41;
  static const gpio_num_t BIN1 = GPIO_NUM_40;
  static const gpio_num_t BIN2 = GPIO_NUM_38;

  static bdc_motor_handle_t bdc_position[NUMS] = {};
  static esp_err_t (*bdc_direction[])(bdc_motor_handle_t) = {
      bdc_motor_forward,
      bdc_motor_reverse,
  };

  void init()
  {
    bdc_motor_config_t motor_cfg = {};
    motor_cfg.pwm_freq_hz = BDC_MCPWM_FREQ_HZ;

    bdc_motor_mcpwm_config_t mcpwm_cfg = {};
    mcpwm_cfg.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

    // left
    motor_cfg.pwma_gpio_num = AIN1;
    motor_cfg.pwmb_gpio_num = AIN2;
    mcpwm_cfg.group_id = 0;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg, &mcpwm_cfg, &bdc_position[LEFT]));

    // right
    motor_cfg.pwma_gpio_num = BIN1;
    motor_cfg.pwmb_gpio_num = BIN2;
    mcpwm_cfg.group_id = 1;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg, &mcpwm_cfg, &bdc_position[RIGHT]));

    // enable
    ESP_ERROR_CHECK(bdc_motor_enable(bdc_position[LEFT]));
    ESP_ERROR_CHECK(bdc_motor_enable(bdc_position[RIGHT]));

    brake();
  }

  void brake()
  {
    brake(LEFT);
    brake(RIGHT);
  }
  void brake(position pos)
  {
    ESP_ERROR_CHECK(bdc_motor_brake(bdc_position[pos]));
  }

  void coast()
  {
    coast(LEFT);
    coast(RIGHT);
  }
  void coast(position pos)
  {
    ESP_ERROR_CHECK(bdc_motor_coast(bdc_position[pos]));
  }

  void duty(position pos, float val)
  {
    direction dir = val < 0.0f ? REVERSE : FORWARD;
    ESP_ERROR_CHECK(bdc_direction[dir](bdc_position[pos]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(bdc_position[pos], static_cast<uint32_t>(static_cast<float>(BDC_MCPWM_DUTY_TICK_MAX) * std::abs(val))));
  }

  std::pair<float, float> duty(std::pair<float, float> val)
  {
    auto [left, right] = val;
    duty(position::LEFT, left);
    duty(position::RIGHT, right);
    return val;
  }
}
