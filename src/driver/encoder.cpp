#include "encoder.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>

#define AS5050A_HOST SPI2_HOST // HSPI
#define AS5050A_PIN_MISO GPIO_NUM_37
#define AS5050A_PIN_MOSI GPIO_NUM_35
#define AS5050A_PIN_SCLK GPIO_NUM_36
#define AS5050A_PIN_CS_LEFT GPIO_NUM_26
#define AS5050A_PIN_CS_RIGHT GPIO_NUM_39

#define AS5050A_REG_POR_OFF 0x3F22
#define AS5050A_REG_SOFTWARE_RESET 0x3C00
#define AS5050A_REG_MASTER_RESET 0x33A5
#define AS5050A_REG_CLEAR_EF 0x3380
#define AS5050A_REG_NOP 0x0000
#define AS5050A_REG_AGC 0x3FF8
#define AS5050A_REG_ANGULAR_DATA 0x3FFF
#define AS5050A_REG_ERROR_STATUS 0x335A
#define AS5050A_REG_SYSTEM_CONFIG 0x3F20

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

  static as5050a_command as5050a_command = {};
  static spi_transaction_t spi_trans_left = {};
  static spi_transaction_t spi_trans_right = {};

  static spi_device_handle_t spi_handle_left = nullptr;
  static spi_device_handle_t spi_handle_right = nullptr;

  static void IRAM_ATTR dma_callback_pre(spi_transaction_t *trans)
  {
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->tx_data[0] = as5050a_command.bytes.msb;
    trans->tx_data[1] = as5050a_command.bytes.lsb;
    trans->length = 16;
  }

  void init()
  {
    spi_bus_config_t spi_bus_cfg = {};
    spi_bus_cfg.mosi_io_num = AS5050A_PIN_MOSI;
    spi_bus_cfg.miso_io_num = AS5050A_PIN_MISO;
    spi_bus_cfg.sclk_io_num = AS5050A_PIN_SCLK;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = 4;
    spi_bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;

    ESP_ERROR_CHECK(spi_bus_initialize(AS5050A_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_dev_if_cfg = {};
    spi_dev_if_cfg.mode = 1;
    spi_dev_if_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    spi_dev_if_cfg.spics_io_num = AS5050A_PIN_CS_LEFT;
    spi_dev_if_cfg.queue_size = 1;
    spi_dev_if_cfg.pre_cb = dma_callback_pre;

    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_left));

    spi_dev_if_cfg.spics_io_num = AS5050A_PIN_CS_RIGHT;
    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_right));

    // resetspi_trans_right
    spi_transaction_t *trans = nullptr;
    as5050a_command.word = as5050a_write(AS5050A_REG_MASTER_RESET);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle_left, &trans, portMAX_DELAY);
    spi_device_get_trans_result(spi_handle_right, &trans, portMAX_DELAY);
  }

  std::pair<uint16_t, uint16_t> angle()
  {
    as5050a_command.word = as5050a_read(AS5050A_REG_ANGULAR_DATA);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));

    spi_transaction_t *trans = nullptr;
    spi_device_get_trans_result(spi_handle_left, &trans, portMAX_DELAY);
    spi_device_get_trans_result(spi_handle_right, &trans, portMAX_DELAY);

    return {as5050a_angle((spi_trans_left.rx_data[0] << 8) | spi_trans_left.rx_data[1]),
            as5050a_angle((spi_trans_right.rx_data[0] << 8) | spi_trans_right.rx_data[1])};
  }
}
