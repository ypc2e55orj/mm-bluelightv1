#include "imu.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>

#include "../third-party/lsm6dsrx-pid/lsm6dsrx_reg.h"

#include <cstdint>
#include <cstring>
#include <cmath>
#include <cassert>

namespace driver::imu
{
  static const spi_host_device_t LSM6DSRX_HOST = SPI3_HOST; // VSPI
  static const gpio_num_t LSM6DSRX_PIN_MISO = GPIO_NUM_48;
  static const gpio_num_t LSM6DSRX_PIN_MOSI = GPIO_NUM_47;
  static const gpio_num_t LSM6DSRX_PIN_SCLK = GPIO_NUM_33;
  static const gpio_num_t LSM6DSRX_PIN_CS = GPIO_NUM_34;

  static const uint32_t LSM6DSRX_BUFFER_SIZE = 16;

  static spi_transaction_t spi_trans = {};
  static spi_device_handle_t spi_handle = {};

  static uint8_t *tx_buffer = nullptr;
  static uint8_t *rx_buffer = nullptr;

  static stmdev_ctx_t lsm6dsrx_ctx = {};

  static EventGroupHandle_t xEvent = nullptr;
  static EventBits_t xEventBit = 0;
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  static int16_t intr_gyro_buff[3] = {};
  static int16_t intr_accel_buff[3] = {};

  static bool initialized = false;

  static void IRAM_ATTR dma_callback_post(spi_transaction_t *trans)
  {
    if (!initialized)
    {
      return;
    }

    intr_gyro_buff[0] = ((int16_t)rx_buffer[2] << 8) | (int16_t)rx_buffer[1];
    intr_gyro_buff[1] = ((int16_t)rx_buffer[4] << 8) | (int16_t)rx_buffer[3];
    intr_gyro_buff[2] = ((int16_t)rx_buffer[6] << 8) | (int16_t)rx_buffer[5];

    intr_accel_buff[0] = ((int16_t)rx_buffer[8] << 8) | (int16_t)rx_buffer[7];
    intr_accel_buff[1] = ((int16_t)rx_buffer[10] << 8) | (int16_t)rx_buffer[9];
    intr_accel_buff[2] = ((int16_t)rx_buffer[12] << 8) | (int16_t)rx_buffer[11];

    xHigherPriorityTaskWoken = pdFALSE;

    if (xEventGroupSetBitsFromISR(xEvent, xEventBit, &xHigherPriorityTaskWoken) == pdPASS)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }

  static int32_t lsm6dsrx_platform_write(void *, uint8_t reg, const uint8_t *bufp, uint16_t len)
  {
    spi_transaction_t *trans = nullptr;
    // write bufp
    tx_buffer[0] = reg;
    memcpy(tx_buffer + 1, bufp, len);

    spi_trans.length = 8 * (len + 1); // reg + data

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY);

    return 0;
  }
  static int32_t lsm6dsrx_platform_read(void *, uint8_t reg, uint8_t *bufp, uint16_t len)
  {
    spi_transaction_t *trans = nullptr;
    // read to bufp
    tx_buffer[0] = reg | 0x80;
    memset(tx_buffer + 1, 0, len);

    spi_trans.length = 8 * (len + 1);
    spi_trans.rxlength = 8 * (len + 1);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY);

    memcpy(bufp, rx_buffer + 1, len);

    return 0;
  }

  void init(EventGroupHandle_t xHandle, EventBits_t xBit)
  {
    xEvent = xHandle;
    xEventBit = xBit;

    spi_bus_config_t spi_bus_cfg = {};
    spi_bus_cfg.mosi_io_num = LSM6DSRX_PIN_MOSI;
    spi_bus_cfg.miso_io_num = LSM6DSRX_PIN_MISO;
    spi_bus_cfg.sclk_io_num = LSM6DSRX_PIN_SCLK;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = LSM6DSRX_BUFFER_SIZE;
    spi_bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;
    spi_bus_cfg.intr_flags = ESP_INTR_FLAG_IRAM;

    ESP_ERROR_CHECK(spi_bus_initialize(LSM6DSRX_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_dev_if_cfg = {};
    spi_dev_if_cfg.mode = 3;
    spi_dev_if_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    spi_dev_if_cfg.spics_io_num = LSM6DSRX_PIN_CS;
    spi_dev_if_cfg.queue_size = 1;
    spi_dev_if_cfg.post_cb = dma_callback_post;

    ESP_ERROR_CHECK(spi_bus_add_device(LSM6DSRX_HOST, &spi_dev_if_cfg, &spi_handle));

    lsm6dsrx_ctx.write_reg = lsm6dsrx_platform_write;
    lsm6dsrx_ctx.read_reg = lsm6dsrx_platform_read;
    lsm6dsrx_ctx.handle = nullptr;

    tx_buffer = (uint8_t *)heap_caps_malloc(LSM6DSRX_BUFFER_SIZE, MALLOC_CAP_DMA);
    rx_buffer = (uint8_t *)heap_caps_malloc(LSM6DSRX_BUFFER_SIZE, MALLOC_CAP_DMA);
    spi_trans.flags = 0;
    spi_trans.tx_buffer = tx_buffer;
    spi_trans.rx_buffer = rx_buffer;

    // settings
    uint8_t id = 0;
    lsm6dsrx_device_id_get(&lsm6dsrx_ctx, &id);
    assert(id == LSM6DSRX_ID);

    lsm6dsrx_reset_set(&lsm6dsrx_ctx, PROPERTY_ENABLE);
    lsm6dsrx_i3c_disable_set(&lsm6dsrx_ctx, LSM6DSRX_I3C_DISABLE);

    lsm6dsrx_block_data_update_set(&lsm6dsrx_ctx, PROPERTY_ENABLE);
    lsm6dsrx_xl_data_rate_set(&lsm6dsrx_ctx, LSM6DSRX_XL_ODR_1666Hz);
    lsm6dsrx_gy_data_rate_set(&lsm6dsrx_ctx, LSM6DSRX_GY_ODR_1666Hz);
    lsm6dsrx_gy_full_scale_set(&lsm6dsrx_ctx, LSM6DSRX_2000dps);

    tx_buffer[0] = LSM6DSRX_OUTX_L_G | 0x80;
    memset(tx_buffer + 1, 0, 12);
    spi_trans.length = 8 * 13;
    spi_trans.rxlength = 8 * 13; // gyro + accel

    initialized = true;
  }

  std::tuple<float, float, float> gyro()
  {
    float gyro_buff[3] = {};

    for (int i = 0; i < 3; i++)
    {
      gyro_buff[i] = (lsm6dsrx_from_fs2000dps_to_mdps(intr_gyro_buff[i]) / 1000.0f) * (M_PI / 180.0f);
    }

    return {gyro_buff[0], gyro_buff[1], gyro_buff[2]};
  }
  std::tuple<int16_t, int16_t, int16_t> gyro_raw()
  {
    return {intr_gyro_buff[0], intr_gyro_buff[1], intr_gyro_buff[2]};
  }

  std::tuple<float, float, float> accel()
  {
    float accel_buff[3] = {};

    for (int i = 0; i < 3; i++)
    {
      accel_buff[i] = (lsm6dsrx_from_fs2g_to_mg(intr_accel_buff[i]) / 1000.0f) * 9.80665f;
    }

    return {accel_buff[0], accel_buff[1], accel_buff[2]};
  }
  std::tuple<int16_t, int16_t, int16_t> accel_raw()
  {
    return {intr_accel_buff[0], intr_accel_buff[1], intr_accel_buff[2]};
  }

  void IRAM_ATTR update()
  {
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
  }
}
