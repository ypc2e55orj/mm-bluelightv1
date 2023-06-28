#include "indicator.h"

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>

#include <led_strip.h>

#include <cstring>

#define WS2812C_PIN GPIO_NUM_45
#define WS2812C_RMT RMT_CHANNEL_0
#define WS2812C_NUM 4
#define WS2812C_COLOR_DEPTH 3
#define WS2812C_RESOLUTION_HZ 10000000

namespace driver::indicator
{
  static led_strip_handle_t led_strip = nullptr;

  uint8_t nums() { return WS2812C_NUM; }

  void init()
  {
    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num = WS2812C_PIN;
    strip_cfg.max_leds = WS2812C_NUM;
    strip_cfg.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_cfg.led_model = LED_MODEL_WS2812;
    strip_cfg.flags.invert_out = false;

    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_cfg.resolution_hz = WS2812C_RESOLUTION_HZ;
    rmt_cfg.flags.with_dma = true;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip));
  }

  void set(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, pos, r, g, b));
  }
  void set(uint8_t pos, uint32_t rgb)
  {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, pos, (rgb & 0xFF0000) >> 16, (rgb & 0xFF00) >> 8, (rgb & 0xFF)));
  }

  void clear()
  {
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
  }

  void update()
  {
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
  }
}
