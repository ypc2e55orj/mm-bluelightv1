#include "indicator.h"

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>

#include "../third-party/led_strip/led_strip_encoder.h"

#include <cstring>

#define WS2812C_PIN GPIO_NUM_45
#define WS2812C_NUM 4
#define WS2812C_COLOR_DEPTH 3
#define WS2812C_RESOLUTION_HZ 10'000'000

namespace driver::indicator
{
  static rmt_channel_handle_t led_chan = nullptr;
  static rmt_encoder_handle_t led_encoder = nullptr;
  static rmt_transmit_config_t tx_config = {};

  static uint8_t pixels[WS2812C_NUM * WS2812C_COLOR_DEPTH] = {};

  uint8_t nums() { return WS2812C_NUM; }

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

  void set(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
  {
    pixels[pos * WS2812C_COLOR_DEPTH] = g;
    pixels[pos * WS2812C_COLOR_DEPTH + 1] = r;
    pixels[pos * WS2812C_COLOR_DEPTH + 2] = b;
  }
  void set(uint8_t pos, uint32_t rgb)
  {
    pixels[pos * WS2812C_COLOR_DEPTH] = (rgb & 0xFF00) >> 8;        // green
    pixels[pos * WS2812C_COLOR_DEPTH + 1] = (rgb & 0xFF0000) >> 16; // red
    pixels[pos * WS2812C_COLOR_DEPTH + 2] = (rgb & 0xFF);           // blue
  }

  void clear()
  {
    memset(pixels, 0, WS2812C_NUM * WS2812C_COLOR_DEPTH);
  }

  void update()
  {
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, pixels, sizeof(pixels), &tx_config));
  }
}
