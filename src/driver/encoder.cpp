#include "encoder.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace driver::encoder
{
  static const spi_host_device_t AS5050A_HOST = SPI2_HOST; // HSPI
  static const gpio_num_t AS5050A_PIN_MISO = GPIO_NUM_37;
  static const gpio_num_t AS5050A_PIN_MOSI = GPIO_NUM_35;
  static const gpio_num_t AS5050A_PIN_SCLK = GPIO_NUM_36;
  static const gpio_num_t AS5050A_PIN_CS_LEFT = GPIO_NUM_26;
  static const gpio_num_t AS5050A_PIN_CS_RIGHT = GPIO_NUM_39;

  static const uint16_t AS5050A_REG_MASTER_RESET = 0x33A5;
  static const uint16_t AS5050A_REG_ANGULAR_DATA = 0x3FFF;

  constexpr uint16_t as5050a_write(uint16_t reg)
  {
    return ((reg << 1) | __builtin_parity(reg));
  }
  constexpr uint16_t as5050a_read(uint16_t reg)
  {
    reg = (reg << 1) | 0x8000;
    return (reg | __builtin_parity(reg));
  }
  inline uint16_t as5050a_angle(uint16_t res)
  {
    return (res & 0x3FFE) >> 2;
  }

  static union as5050a_command {
    uint16_t word;
    struct
    {
      uint8_t lsb;
      uint8_t msb;
    } bytes;
  } as5050a_command = {};

  DMA_ATTR static spi_transaction_t spi_trans_left = {};
  DMA_ATTR static spi_transaction_t spi_trans_right = {};

  static spi_device_handle_t spi_handle_left = nullptr;
  static spi_device_handle_t spi_handle_right = nullptr;

  static EventGroupHandle_t xEvent = nullptr;
  static EventBits_t xEventBit = 0;
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  static uint16_t angle_left = 0;
  static uint16_t angle_right = 0;

  static bool initialized = false;

  static void IRAM_ATTR dma_callback_pre(spi_transaction_t *trans)
  {
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->tx_data[0] = as5050a_command.bytes.msb;
    trans->tx_data[1] = as5050a_command.bytes.lsb;
    trans->length = 16;
  }

  static void IRAM_ATTR dma_callback_post(spi_transaction_t *trans)
  {
    if (!initialized)
    {
      return;
    }

    uint16_t rx_data = (trans->rx_data[0] << 8) | trans->rx_data[1];
    xHigherPriorityTaskWoken = pdFALSE;

    if (trans == &spi_trans_left)
    {
      angle_left = as5050a_angle(rx_data);
    }
    else // if (trans == &spi_trans_right)
    {
      angle_right = as5050a_angle(rx_data);
      if (xEventGroupSetBitsFromISR(xEvent, xEventBit, &xHigherPriorityTaskWoken) == pdPASS)
      {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }

  void init(EventGroupHandle_t xHandle, EventBits_t xBit)
  {
    // notify handle
    xEvent = xHandle;
    xEventBit = xBit;

    // bus
    spi_bus_config_t spi_bus_cfg = {};
    spi_bus_cfg.mosi_io_num = AS5050A_PIN_MOSI;
    spi_bus_cfg.miso_io_num = AS5050A_PIN_MISO;
    spi_bus_cfg.sclk_io_num = AS5050A_PIN_SCLK;
    spi_bus_cfg.max_transfer_sz = 4;
    spi_bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;
    spi_bus_cfg.intr_flags = ESP_INTR_FLAG_IRAM;

    ESP_ERROR_CHECK(spi_bus_initialize(AS5050A_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    // device
    spi_device_interface_config_t spi_dev_if_cfg = {};
    spi_dev_if_cfg.mode = 1;
    spi_dev_if_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    spi_dev_if_cfg.spics_io_num = AS5050A_PIN_CS_LEFT;
    spi_dev_if_cfg.queue_size = 1;
    spi_dev_if_cfg.pre_cb = dma_callback_pre;
    spi_dev_if_cfg.post_cb = dma_callback_post;

    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_left));

    spi_dev_if_cfg.spics_io_num = AS5050A_PIN_CS_RIGHT;
    ESP_ERROR_CHECK(spi_bus_add_device(AS5050A_HOST, &spi_dev_if_cfg, &spi_handle_right));

    // reset
    spi_transaction_t *trans = nullptr;
    as5050a_command.word = as5050a_write(AS5050A_REG_MASTER_RESET);
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle_left, &trans, portMAX_DELAY);
    spi_device_get_trans_result(spi_handle_right, &trans, portMAX_DELAY);

    as5050a_command.word = as5050a_read(AS5050A_REG_ANGULAR_DATA);

    initialized = true;
  }

  void IRAM_ATTR update()
  {
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_left, &spi_trans_left, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle_right, &spi_trans_right, portMAX_DELAY));
  }

  std::pair<uint16_t, uint16_t> get()
  {
    return {angle_left, angle_right};
  }
}
