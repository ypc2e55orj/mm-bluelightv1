#include "imu.hpp"

// C++
#include <bitset>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>

// Project
#include "base.hpp"
#include "lsm6dsrx_reg.h"

namespace driver::hardware
{
  static const auto TAG = "driver::hardware::Imu";

  class Imu::Lsm6dsrxImpl final : DriverBase
  {
  private:
    peripherals::Spi &spi_;
    int index_;
    uint8_t *rx_buffer_;
    uint8_t *tx_buffer_;

    Axis gyro_;
    Axis accel_;

    // 送受信バッファサイズ
    static constexpr size_t BUFFER_SIZE = 12;

    // レジスタ
    static constexpr uint8_t REG_WHO_AM_I = 0x0F;

    static constexpr uint8_t REG_CTRL3_C = 0x12;
    static constexpr uint8_t BIT_CTRL3_C_SW_RESET = 0;
    static constexpr uint8_t BIT_CTRL3_C_BDU = 6;

    static constexpr uint8_t REG_CTRL9_XL = 0x18;
    static constexpr uint8_t BIT_CTRL3_I3C_DISABLE = 1;

    static constexpr uint8_t REG_CTRL1_XL = 0x10;
    static constexpr uint8_t BIT_CTRL1_XL_ODR_XL3 = 7;
    static constexpr uint8_t BIT_CTRL1_XL_ODR_XL2 = 6;
    static constexpr uint8_t BIT_CTRL1_XL_ODR_XL1 = 5;
    static constexpr uint8_t BIT_CTRL1_XL_ODR_XL0 = 4;

    static constexpr uint8_t REG_CTRL2_G = 0x11;
    static constexpr uint8_t BIT_CTRL2_G_ODR_G3 = 7;
    static constexpr uint8_t BIT_CTRL2_G_ODR_G2 = 6;
    static constexpr uint8_t BIT_CTRL2_G_ODR_G1 = 5;
    static constexpr uint8_t BIT_CTRL2_G_ODR_G0 = 4;
    static constexpr uint8_t BIT_CTRL2_G_FS1_G = 3;
    static constexpr uint8_t BIT_CTRL2_G_FS0_G = 2;
    static constexpr uint8_t BIT_CTRL2_G_FS_125 = 1;
    static constexpr uint8_t BIT_CTRL2_G_FS_4000 = 0;

    static constexpr uint8_t REG_OUTX_L_G = 0x22;

    uint8_t read_byte(uint8_t reg)
    {
      auto trans = spi_.transaction(index_);
      uint8_t *p = trans->rx_data;
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->length = 8;
      trans->addr = reg | 0x80;
      spi_.transmit(index_);
      ESP_LOGI(TAG, "read_byte(%02x): %02x %02x %02x %02x", reg, p[0], p[1], p[2], p[3]);
      return p[0];
    }
    bool write_byte(uint8_t reg, uint8_t data)
    {
      auto trans = spi_.transaction(index_);
      uint8_t *p = trans->rx_data;
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->length = 8;
      trans->addr = reg;
      trans->tx_data[0] = data;
      bool ret = spi_.transmit(index_);
      ESP_LOGI(TAG, "write_byte(%02x, %02x): %02x %02x %02x %02x", reg, data, p[0], p[1], p[2], p[3]);
      return ret;
    }

  public:
    explicit Lsm6dsrxImpl(peripherals::Spi &spi, gpio_num_t spics_io_num) : spi_(spi), gyro_(), accel_()
    {
      // 転送用バッファを確保
      tx_buffer_ = reinterpret_cast<uint8_t *>(heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));
      rx_buffer_ = reinterpret_cast<uint8_t *>(heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));

      // バスにデバイスを追加
      index_ = spi_.add(0, 8, 3, SPI_MASTER_FREQ_10M, spics_io_num, 1);
      // 初期設定
      std::bitset<8> reg;
      // IDを取得
      ESP_LOGI(TAG, "WHO_AM_I(%02x)", read_byte(REG_WHO_AM_I));
      // ソフトウェア・リセット
      reg = read_byte(REG_CTRL3_C);
      reg[BIT_CTRL3_C_SW_RESET] = true;
      write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));
      read_byte(REG_CTRL3_C);
      // I3Cを無効化
      reg = read_byte(REG_CTRL9_XL);
      reg[BIT_CTRL3_I3C_DISABLE] = true;
      write_byte(REG_CTRL9_XL, static_cast<uint8_t>(reg.to_ulong()));
      read_byte(REG_CTRL9_XL);
      // 読み出ししているレジスタは更新しない(Block Data Update)
      reg = read_byte(REG_CTRL3_C);
      reg[BIT_CTRL3_C_BDU] = true;
      write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));
      read_byte(REG_CTRL3_C);
      // 加速度計の更新レートを設定(1.66kHz)
      reg = read_byte(REG_CTRL1_XL);
      reg[BIT_CTRL1_XL_ODR_XL3] = true;
      reg[BIT_CTRL1_XL_ODR_XL2] = false;
      reg[BIT_CTRL1_XL_ODR_XL1] = false;
      reg[BIT_CTRL1_XL_ODR_XL0] = false;
      write_byte(REG_CTRL1_XL, static_cast<uint8_t>(reg.to_ulong()));
      read_byte(REG_CTRL1_XL);
      // 角速度計の更新レートを設定(1.66kHz)、スケールを設定(+-2000dps)
      reg = read_byte(REG_CTRL2_G);
      reg[BIT_CTRL2_G_ODR_G3] = true;
      reg[BIT_CTRL2_G_ODR_G2] = false;
      reg[BIT_CTRL2_G_ODR_G1] = false;
      reg[BIT_CTRL2_G_ODR_G0] = false;
      reg[BIT_CTRL2_G_FS1_G] = true;
      reg[BIT_CTRL2_G_FS0_G] = false;
      reg[BIT_CTRL2_G_FS_125] = false;
      reg[BIT_CTRL2_G_FS_4000] = false;
      write_byte(REG_CTRL2_G, static_cast<uint8_t>(reg.to_ulong()));
      read_byte(REG_CTRL2_G);

      // 更新
      auto trans = spi_.transaction(index_);
      trans->flags = 0;
      trans->tx_buffer = tx_buffer_;
      trans->rx_buffer = rx_buffer_;
      trans->addr = REG_OUTX_L_G | 0x80;
      trans->length = 12 * 8; // OUTX_L_G(22h) ~ OUTZ_H_A(2Dh)
      trans->rxlength = trans->length;
      spi_.transmit(index_);
    }
    ~Lsm6dsrxImpl()
    {
      free(tx_buffer_);
      free(rx_buffer_);
    }

    bool update() override
    {
      return spi_.transmit(index_);
    }

    Axis &gyro()
    {
      constexpr float gyro_const = 70.0f; // 2000dps
      auto res = reinterpret_cast<int16_t *>(rx_buffer_);
      gyro_.x = static_cast<float>(res[0]) * gyro_const;
      gyro_.y = static_cast<float>(res[1]) * gyro_const;
      gyro_.z = static_cast<float>(res[2]) * gyro_const;
      return gyro_;
    }

    Axis &accel()
    {
      constexpr float accel_const = 9.80665f * 0.061f; // fs2g
      auto res = reinterpret_cast<int16_t *>(rx_buffer_);
      accel_.x = static_cast<float>(res[3]) * accel_const;
      accel_.y = static_cast<float>(res[4]) * accel_const;
      accel_.z = static_cast<float>(res[5]) * accel_const;
      return accel_;
    }
  };

  Imu::Imu(peripherals::Spi &spi, gpio_num_t spics_io_num) : impl_(new Lsm6dsrxImpl(spi, spics_io_num))
  {
  }
  Imu::~Imu() = default;

  bool Imu::update()
  {
    return impl_->update();
  }
  Imu::Axis &Imu::gyro()
  {
    return impl_->gyro();
  }
  Imu::Axis &Imu::accel()
  {
    return impl_->accel();
  }
}

namespace driver::imu
{
  constexpr spi_host_device_t LSM6DSRX_HOST = SPI3_HOST; // VSPI
  constexpr gpio_num_t LSM6DSRX_PIN_MISO = GPIO_NUM_48;
  constexpr gpio_num_t LSM6DSRX_PIN_MOSI = GPIO_NUM_47;
  constexpr gpio_num_t LSM6DSRX_PIN_SCLK = GPIO_NUM_33;
  constexpr gpio_num_t LSM6DSRX_PIN_CS = GPIO_NUM_34;
  constexpr uint32_t LSM6DSRX_BUFFER_SIZE = 16;

  static stmdev_ctx_t lsm6dsrx_ctx = {};

  static DRAM_ATTR spi_transaction_t spi_trans = {};
  static DRAM_ATTR spi_device_handle_t spi_handle = {};

  static DRAM_ATTR uint8_t *tx_buffer = nullptr;
  static DRAM_ATTR uint8_t *rx_buffer = nullptr;

  static DRAM_ATTR int16_t intr_gyro_buff[3] = {};
  static DRAM_ATTR int16_t intr_accel_buff[3] = {};

  static DRAM_ATTR bool initialized = false;

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
  }

  static int32_t lsm6dsrx_platform_write(void *, uint8_t reg, const uint8_t *bufp, uint16_t len)
  {
    spi_transaction_t *trans = nullptr;
    // write bufp
    tx_buffer[0] = reg;

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

    spi_trans.length = 8 * (len + 1);
    spi_trans.rxlength = 8 * (len + 1);

    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
    spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY);

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

  std::tuple<float, float, float> accel()
  {
    float accel_buff[3] = {};

    for (int i = 0; i < 3; i++)
    {
      accel_buff[i] = (lsm6dsrx_from_fs2g_to_mg(intr_accel_buff[i]) / 1000.0f) * 9.80665f;
    }

    return {accel_buff[0], accel_buff[1], accel_buff[2]};
  }

  void IRAM_ATTR update()
  {
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_handle, &spi_trans, portMAX_DELAY));
  }

  void IRAM_ATTR wait()
  {
    spi_transaction_t *trans;
    ESP_ERROR_CHECK(spi_device_get_trans_result(spi_handle, &trans, portMAX_DELAY));
  }
}
