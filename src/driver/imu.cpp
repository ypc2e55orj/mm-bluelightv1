#include "imu.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "../third-party/lsm6dsrx-pid/lsm6dsrx_reg.h"

#include <cstring>

#define LSM6DSRX_HOST SPI3_HOST // VSPI
#define LSM6DSRX_MISO GPIO_NUM_48
#define LSM6DSRX_MOSI GPIO_NUM_47
#define LSM6DSRX_SCLK GPIO_NUM_33
#define LSM6DSRX_CS GPIO_NUM_34

#define LSM6DSRX_TRANS_MAX 16

namespace driver::imu
{
  static spi_transaction_t spi_trans = {0};
  static spi_device_handle_t spi_handle = {0};

  static uint8_t *tx_buffer = nullptr;
  static uint8_t *rx_buffer = nullptr;

  static stmdev_ctx_t lsm6dsrx_ctx = {0};

  static void dma_callback_pre(spi_transaction_t *trans)
  {
  }
  static void dma_callback_post(spi_transaction_t *trans)
  {
  }

  static int32_t lsm6dsrx_platform_write(void *unused, uint8_t reg, const uint8_t *bufp, uint16_t len)
  {
    spi_transaction_t *trans;

    memset(tx_buffer, 0, LSM6DSRX_TRANS_MAX);
    memset(rx_buffer, 0, LSM6DSRX_TRANS_MAX);
    spi_trans.flags = 0;
    spi_trans.rx_buffer = rx_buffer;
    spi_trans.tx_buffer = tx_buffer;
    spi_trans.length = 8 * (len + 1);

    tx_buffer[0] = reg;
    memcpy(tx_buffer + 1, bufp, len);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY);

    return 0;
  }
  static int32_t lsm6dsrx_platform_read(void *unused, uint8_t reg, uint8_t *bufp, uint16_t len)
  {
    spi_transaction_t *trans;

    memset(tx_buffer, 0, LSM6DSRX_TRANS_MAX);
    memset(rx_buffer, 0, LSM6DSRX_TRANS_MAX);
    spi_trans.flags = 0;
    spi_trans.rx_buffer = rx_buffer;
    spi_trans.tx_buffer = tx_buffer;
    spi_trans.length = 8 * (len + 1);

    tx_buffer[0] = reg | 0x80;

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY);

    memcpy(bufp, rx_buffer + 1, len);

    return 0;
  }

  void init()
  {
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = LSM6DSRX_MOSI,
        .miso_io_num = LSM6DSRX_MISO,
        .sclk_io_num = LSM6DSRX_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LSM6DSRX_TRANS_MAX,
        .flags = SPICOMMON_BUSFLAG_MASTER};

    ESP_ERROR_CHECK(spi_bus_initialize(LSM6DSRX_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_dev_if_cfg = {
        .mode = 3,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = LSM6DSRX_CS,
        .queue_size = 1,
        .pre_cb = dma_callback_pre,
        .post_cb = dma_callback_post};

    ESP_ERROR_CHECK(spi_bus_add_device(LSM6DSRX_HOST, &spi_dev_if_cfg, &spi_handle));

    lsm6dsrx_ctx.write_reg = lsm6dsrx_platform_write;
    lsm6dsrx_ctx.read_reg = lsm6dsrx_platform_read;
    lsm6dsrx_ctx.handle = nullptr;

    tx_buffer = (uint8_t *)heap_caps_malloc(LSM6DSRX_TRANS_MAX, MALLOC_CAP_DMA);
    rx_buffer = (uint8_t *)heap_caps_malloc(LSM6DSRX_TRANS_MAX, MALLOC_CAP_DMA);
  }
}
