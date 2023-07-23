#include "indicator.h"

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>

#include "../third-party/led_strip/led_strip_encoder.h"

#include <cstring>

namespace driver::indicator
{
  static const gpio_num_t WS2812C_PIN = GPIO_NUM_45;
  static const uint32_t WS2812C_NUM = 4;
  static const uint32_t WS2812C_COLOR_DEPTH = 3;
  static const uint32_t WS2812C_RESOLUTION_HZ = 10'000'000;

  static rmt_channel_handle_t led_chan = nullptr;
  static rmt_encoder_handle_t led_encoder = nullptr;
  static rmt_transmit_config_t tx_config = {};

  static uint8_t pixels[WS2812C_NUM * WS2812C_COLOR_DEPTH] = {};

  const uint8_t nums()
  {
    return WS2812C_NUM;
  }

  void init()
  {
    rmt_tx_channel_config_t tx_chan_cfg = {};
    tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_cfg.gpio_num = WS2812C_PIN;
    tx_chan_cfg.mem_block_symbols = 64;
    tx_chan_cfg.resolution_hz = WS2812C_RESOLUTION_HZ;
    tx_chan_cfg.trans_queue_depth = 4;
    tx_chan_cfg.flags.with_dma = 1;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &led_chan));

    led_strip_encoder_config_t encoder_cfg = {};
    encoder_cfg.resolution = WS2812C_RESOLUTION_HZ;

    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_cfg, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));
  }

  void update()
  {
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, pixels, sizeof(pixels), &tx_config));
  }

  void set(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
  {
    pixels[pos * WS2812C_COLOR_DEPTH] = g;
    pixels[pos * WS2812C_COLOR_DEPTH + 1] = r;
    pixels[pos * WS2812C_COLOR_DEPTH + 2] = b;
  }
  void set(uint8_t pos, uint32_t rgb)
  {
    set(pos, (rgb & 0xFF0000) >> 16, (rgb & 0xFF00) >> 8, (rgb & 0xFF));
  }

  // https://github.com/adafruit/Adafruit_NeoPixel/blob/9cf5e96e8f436cc3460f6e7a32b20aceab6e905c/examples/strandtest_wheel/strandtest_wheel.ino
  void rainbow_yield(bool reset)
  {
    static uint8_t p = 0;
    if (reset)
    {
      p = 0;
    }

    const auto wheel = [](uint8_t pos) -> uint32_t {
      if (pos < 85)
      {
        return ((255 - pos * 3) << 16) | (pos * 3);
      }
      if (pos < 170)
      {
        pos -= 85;
        return ((pos * 3) << 8) | (255 - pos * 3);
      }
      pos -= 170;
      return ((pos * 3) << 16) | ((255 - pos * 3) << 8);
    };

    for (int i = 0; i < nums(); i++)
    {
      set(i, wheel((i + p) & 0xFF));
    }
    p++;
  }

  void clear()
  {
    memset(pixels, 0, WS2812C_NUM * WS2812C_COLOR_DEPTH);
  }
}
