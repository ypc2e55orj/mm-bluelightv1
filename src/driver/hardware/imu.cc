#include "imu.h"

// C++
#include <bitset>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>

// Project
#include "base.h"

namespace driver::hardware {

class Imu::Lsm6dsrxImpl final : public DriverBase {
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
  static constexpr uint8_t DAT_WHO_AM_I = 0x6B;

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

  uint8_t read_byte(uint8_t reg) {
    auto trans = spi_.transaction(index_);
    uint8_t *p = trans->rx_data;
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->length = 8;
    trans->addr = reg | 0x80;
    spi_.transmit(index_);
    return p[0];
  }
  bool write_byte(uint8_t reg, uint8_t data) {
    auto trans = spi_.transaction(index_);
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->length = 8;
    trans->addr = reg;
    trans->tx_data[0] = data;
    bool ret = spi_.transmit(index_);
    return ret;
  }

 public:
  explicit Lsm6dsrxImpl(peripherals::Spi &spi, gpio_num_t spics_io_num)
      : spi_(spi), gyro_(), accel_() {
    // 転送用バッファを確保
    tx_buffer_ = reinterpret_cast<uint8_t *>(
        heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));
    rx_buffer_ = reinterpret_cast<uint8_t *>(
        heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));

    // バスにデバイスを追加
    index_ = spi_.add(0, 8, 3, SPI_MASTER_FREQ_10M, spics_io_num, 1);
    // 初期設定
    std::bitset<8> reg;
    // IDを取得
    assert(read_byte(REG_WHO_AM_I) == DAT_WHO_AM_I);
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
    reg[BIT_CTRL2_G_FS0_G] = true;
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
    trans->length = 12 * 8;  // OUTX_L_G(22h) ~ OUTZ_H_A(2Dh)
    trans->rxlength = trans->length;
    spi_.transmit(index_);
  }
  ~Lsm6dsrxImpl() {
    free(tx_buffer_);
    free(rx_buffer_);
  }

  bool update() override { return spi_.transmit(index_); }

  const Axis &gyro() {
    constexpr float gyro_const = 70.0f;  // 2000dps
    auto res = reinterpret_cast<int16_t *>(rx_buffer_);
    gyro_.x = static_cast<float>(res[0]) * gyro_const;
    gyro_.y = static_cast<float>(res[1]) * gyro_const;
    gyro_.z = static_cast<float>(res[2]) * gyro_const;
    return gyro_;
  }

  const Axis &accel() {
    constexpr float accel_const = 9.80665f * 0.061f;  // fs2g
    auto res = reinterpret_cast<int16_t *>(rx_buffer_);
    accel_.x = static_cast<float>(res[3]) * accel_const;
    accel_.y = static_cast<float>(res[4]) * accel_const;
    accel_.z = static_cast<float>(res[5]) * accel_const;
    return accel_;
  }
};

Imu::Imu(peripherals::Spi &spi, gpio_num_t spics_io_num)
    : impl_(new Lsm6dsrxImpl(spi, spics_io_num)) {}
Imu::~Imu() = default;

bool Imu::update() { return impl_->update(); }
const Imu::Axis &Imu::gyro() { return impl_->gyro(); }
const Imu::Axis &Imu::accel() { return impl_->accel(); }
}  // namespace driver::hardware
