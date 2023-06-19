#include "encoder.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>

#define AS5050A_HOST SPI2_HOST // HSPI
#define AS5050A_MISO GPIO_NUM_37
#define AS5050A_MOSI GPIO_NUM_35
#define AS5050A_SCLK GPIO_NUM_36
#define AS5050A_CS_LEFT GPIO_NUM_26
#define AS5050A_CS_RIGHT GPIO_NUM_39

#define AS5050A_POR_OFF 0x3F22
#define AS5050A_SOFTWARE_RESET 0x3C00
#define AS5050A_MASTER_RESET 0x33A5
#define AS5050A_CLEAR_EF 0x3380
#define AS5050A_NOP 0x0000
#define AS5050A_AGC 0x3FF8
#define AS5050A_ANGULAR_DATA 0x3FFF
#define AS5050A_ERROR_STATUS 0x335A
#define AS5050A_SYSTEM_CONFIG 0x3F20

namespace driver::encoder
{
  union as5050a_command
  {
    uint16_t word;
    struct
    {
      uint8_t lsb;
      uint8_t msb;
    } bytes;
  };
  inline uint16_t as5050a_write(uint16_t reg)
  {
    return ((reg << 1) | __builtin_parity(reg));
  }
  inline uint16_t as5050a_read(uint16_t reg)
  {
    reg = (reg << 1) | 0x8000;
    return (reg | __builtin_parity(reg));
  }
  inline uint16_t as5050a_angle(uint16_t res)
  {
    return (res & 0x3FFE) >> 2;
  }
  inline bool as5050a_parity(uint16_t res)
  {
    return __builtin_parity(res & (~0x1)) != (res & 0x1);
  }
  inline bool as5050a_alarm_lo(uint16_t res)
  {
    return (res & 0x8000) != 0;
  }
  inline bool as5050a_alarm_hi(uint16_t res)
  {
    return (res & 0x4000) != 0;
  }

  static as5050a_command as5050a_command;
  static spi_transaction_t spi_trans_left = {0};
  static spi_transaction_t spi_trans_right = {0};

  static spi_device_handle_t spi_handle_left = {0};
  static spi_device_handle_t spi_handle_right = {0};

  static uint16_t angle_left = 0;
  static bool error_parity_left = false;
  static bool error_alarm_lo_left = false;
  static bool error_alarm_hi_left = false;

  static uint16_t angle_right = 0;
  static bool error_parity_right = false;
  static bool error_alarm_lo_right = false;
  static bool error_alarm_hi_right = false;

  static void dma_callback_pre(spi_transaction_t *trans)
  {
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->tx_data[0] = as5050a_command.bytes.msb;
    trans->tx_data[1] = as5050a_command.bytes.lsb;
    trans->length = 16;
  }
  static void dma_callback_post(spi_transaction_t *trans)
  {
  }

  void init()
  {
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = AS5050A_MOSI,
        .miso_io_num = AS5050A_MISO,
        .sclk_io_num = AS5050A_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
        .flags = SPICOMMON_BUSFLAG_MASTER};

    ESP_ERROR_CHECK(spi_bus_initialize(AS5050A_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_dev_if_cfg = {
        .mode = 1,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = AS5050A_CS_LEFT,
        .queue_size = 1,
        .pre_cb = dma_callback_pre,
        .post_cb = dma_callback_post};

    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_left));

    spi_dev_if_cfg.spics_io_num = AS5050A_CS_RIGHT;
    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_right));

    spi_transaction_t *trans;
    as5050a_command.word = as5050a_write(AS5050A_MASTER_RESET);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle_left, &trans, portMAX_DELAY);
    spi_device_get_trans_result(spi_handle_right, &trans, portMAX_DELAY);
  }

  void angle(uint16_t &left, uint16_t &right)
  {
    as5050a_command.word = as5050a_read(AS5050A_ANGULAR_DATA);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));

    spi_transaction_t *trans;
    spi_device_get_trans_result(spi_handle_left, &trans, portMAX_DELAY);
    spi_device_get_trans_result(spi_handle_right, &trans, portMAX_DELAY);

    uint16_t res;
    res = (spi_trans_left.rx_data[0] << 8) | spi_trans_left.rx_data[1];
    left = angle_left = as5050a_angle(res);
    error_parity_left = as5050a_parity(res);
    error_alarm_hi_left = as5050a_alarm_hi(res);
    error_alarm_lo_left = as5050a_alarm_lo(res);

    res = (spi_trans_right.rx_data[0] << 8) | spi_trans_right.rx_data[1];
    right = angle_right = as5050a_angle(res);
    error_parity_right = as5050a_parity(res);
    error_alarm_hi_right = as5050a_alarm_hi(res);
    error_alarm_lo_right = as5050a_alarm_lo(res);
  }

  std::pair<uint16_t, uint16_t> angle()
  {
    uint16_t left, right;
    angle(left, right);
    return {left, right};
  }
}
