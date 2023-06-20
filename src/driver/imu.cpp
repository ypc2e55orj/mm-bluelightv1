#include "imu.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "../third-party/lsm6dsrx-pid/lsm6dsrx_reg.h"

#include <cstdint>
#include <cstring>
#include <cassert>

#define LSM6DSRX_HOST SPI3_HOST // VSPI
#define LSM6DSRX_PIN_MISO GPIO_NUM_48
#define LSM6DSRX_PIN_MOSI GPIO_NUM_47
#define LSM6DSRX_PIN_SCLK GPIO_NUM_33
#define LSM6DSRX_PIN_CS GPIO_NUM_34

#define LSM6DSRX_BUFFER_SIZE 16

namespace driver::imu
{
  static spi_transaction_t spi_trans = {};
  static spi_device_handle_t spi_handle = {};

  static uint8_t *tx_buffer = nullptr;
  static uint8_t *rx_buffer = nullptr;

  static stmdev_ctx_t lsm6dsrx_ctx = {};

  static int32_t lsm6dsrx_platform_write(void *unused, uint8_t reg, const uint8_t *bufp, uint16_t len)
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
  static int32_t lsm6dsrx_platform_read(void *unused, uint8_t reg, uint8_t *bufp, uint16_t len)
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

  void init()
  {
    spi_bus_config_t spi_bus_cfg = {};
    spi_bus_cfg.mosi_io_num = LSM6DSRX_PIN_MOSI;
    spi_bus_cfg.miso_io_num = LSM6DSRX_PIN_MISO;
    spi_bus_cfg.sclk_io_num = LSM6DSRX_PIN_SCLK;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = LSM6DSRX_BUFFER_SIZE;
    spi_bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;

    ESP_ERROR_CHECK(spi_bus_initialize(LSM6DSRX_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_dev_if_cfg = {};
    spi_dev_if_cfg.mode = 3;
    spi_dev_if_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;
    spi_dev_if_cfg.spics_io_num = LSM6DSRX_PIN_CS;
    spi_dev_if_cfg.queue_size = 1;

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
  }

  std::tuple<float, float, float> gyro()
  {
    int16_t val[3] = {};
    lsm6dsrx_angular_rate_raw_get(&lsm6dsrx_ctx, val);
    return {lsm6dsrx_from_fs2000dps_to_mdps(val[0]), lsm6dsrx_from_fs2000dps_to_mdps(val[1]), lsm6dsrx_from_fs2000dps_to_mdps(val[2])};
  }
  std::tuple<float, float, float> accel()
  {
    int16_t val[3] = {};
    lsm6dsrx_acceleration_raw_get(&lsm6dsrx_ctx, val);
    return {lsm6dsrx_from_fs2g_to_mg(val[0]), lsm6dsrx_from_fs2g_to_mg(val[1]), lsm6dsrx_from_fs2g_to_mg(val[2])};
  }
}
