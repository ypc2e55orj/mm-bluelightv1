#include "buzzer.h"

#include <driver/gpio.h>
#include <driver/rmt_tx.h>

#include "../third-party/musical_buzzer/musical_score_encoder.h"

#define BUZZER_PIN GPIO_NUM_21
#define BUZZER_RESOLUTION_HZ 1000000

namespace driver::buzzer
{
  static rmt_channel_handle_t buzzer_chan = nullptr;
  static rmt_encoder_handle_t score_encoder = nullptr;
  static rmt_transmit_config_t tx_config = {};

  static buzzer_musical_score_t score = {};

  void init()
  {
    rmt_tx_channel_config_t tx_chan_cfg = {};
    tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_cfg.gpio_num = BUZZER_PIN;
    tx_chan_cfg.mem_block_symbols = 64;
    tx_chan_cfg.resolution_hz = BUZZER_RESOLUTION_HZ;
    tx_chan_cfg.trans_queue_depth = 4;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &buzzer_chan));

    musical_score_encoder_config_t encoder_cfg = {};
    encoder_cfg.resolution = BUZZER_RESOLUTION_HZ;

    ESP_ERROR_CHECK(rmt_new_musical_score_encoder(&encoder_cfg, &score_encoder));
    ESP_ERROR_CHECK(rmt_enable(buzzer_chan));
  }

  void tone(uint32_t freq, uint32_t ms)
  {
    score.duration_ms = ms;
    score.freq_hz = freq;
    tx_config.loop_count = ms * freq / 1000;

    ESP_ERROR_CHECK(rmt_transmit(buzzer_chan, score_encoder, &score, sizeof(buzzer_musical_score_t), &tx_config));
  }
}
