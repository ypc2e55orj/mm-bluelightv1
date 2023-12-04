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

  RawAxis gyro_;
  RawAxis accel_;

  // 送受信バッファサイズ
  static constexpr size_t BUFFER_SIZE = 12;

  // レジスタ
  static constexpr uint8_t REG_WHO_AM_I = 0x0F;
  static constexpr uint8_t DAT_WHO_AM_I = 0x6B;

  static constexpr uint8_t REG_CTRL1_XL = 0x10;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL3 = 7;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL2 = 6;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL1 = 5;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL0 = 4;
  static constexpr uint8_t BIT_CTRL1_XL_FS1_XL = 3;
  static constexpr uint8_t BIT_CTRL1_XL_FS0_XL = 2;
  static constexpr uint8_t BIT_CTRL1_XL_LPF2_XL_EN = 1;

  static constexpr uint8_t REG_CTRL2_G = 0x11;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G3 = 7;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G2 = 6;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G1 = 5;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G0 = 4;
  static constexpr uint8_t BIT_CTRL2_G_FS1_G = 3;
  static constexpr uint8_t BIT_CTRL2_G_FS0_G = 2;
  static constexpr uint8_t BIT_CTRL2_G_FS_125 = 1;
  static constexpr uint8_t BIT_CTRL2_G_FS_4000 = 0;

  static constexpr uint8_t REG_CTRL3_C = 0x12;
  static constexpr uint8_t BIT_CTRL3_C_BDU = 6;
  static constexpr uint8_t BIT_CTRL3_C_SW_RESET = 0;

  static constexpr uint8_t REG_CTRL4_C = 0x13;
  static constexpr uint8_t BIT_CTRL4_C_LPF1_SEL_G = 1;

  static constexpr uint8_t REG_CTRL6_C = 0x15;
  static constexpr uint8_t BIT_CTRL6_C_USR_OFF_W = 3;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_2 = 2;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_1 = 1;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_0 = 0;

  static constexpr uint8_t REG_CTRL7_G = 0x16;
  static constexpr uint8_t BIT_CTRL7_G_USR_OFF_ON_OUT = 1;

  static constexpr uint8_t REG_CTRL8_XL = 0x17;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_2 = 7;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_1 = 6;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_0 = 5;
  static constexpr uint8_t BIT_CTRL8_XL_FASTSETTL_MODE_XL = 3;

  static constexpr uint8_t REG_CTRL9_XL = 0x18;
  static constexpr uint8_t BIT_CTRL9_XL_I3C_DISABLE = 1;

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

    std::bitset<8> reg;
    // 相手が合っているか確認
    assert(read_byte(REG_WHO_AM_I) == DAT_WHO_AM_I);
    reg = read_byte(REG_CTRL3_C);
    // ソフトウェア・リセット
    reg[BIT_CTRL3_C_SW_RESET] = true;
    // リセット実行
    write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));

    // 初期設定
    reg = read_byte(REG_CTRL9_XL);
    // I3Cを無効化
    reg[BIT_CTRL9_XL_I3C_DISABLE] = true;
    // CTRL9_XLを反映
    write_byte(REG_CTRL9_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL3_C);
    // 読み出ししているレジスタは更新しない(Block Data Update)
    reg[BIT_CTRL3_C_BDU] = true;
    // CTRL3_Cを反映
    write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));

    // 加速度計の設定
    reg = read_byte(REG_CTRL1_XL);
    // 出力レートを1.66kHzに設定
    reg[BIT_CTRL1_XL_ODR_XL3] = true;
    reg[BIT_CTRL1_XL_ODR_XL2] = false;
    reg[BIT_CTRL1_XL_ODR_XL1] = false;
    reg[BIT_CTRL1_XL_ODR_XL0] = false;
    // スケールを+-2gに設定
    reg[BIT_CTRL1_XL_FS1_XL] = false;
    reg[BIT_CTRL1_XL_FS0_XL] = false;
    // LPF2を有効
    reg[BIT_CTRL1_XL_LPF2_XL_EN] = true;
    // CTRL1_XLを反映
    write_byte(REG_CTRL1_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL8_XL);
    // フィルタをLow pass, ODR/10に設定
    reg[BIT_CTRL8_XL_HPCF_XL_2] = false;
    reg[BIT_CTRL8_XL_HPCF_XL_1] = false;
    reg[BIT_CTRL8_XL_HPCF_XL_0] = true;
    reg[BIT_CTRL8_XL_FASTSETTL_MODE_XL] = true;
    // CTRL8_XLを反映
    write_byte(REG_CTRL8_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL6_C);
    // オフセットの重みを2^-6 g/LSBに設定
    reg[BIT_CTRL6_C_USR_OFF_W] = true;
    // CTRL6_Cを反映
    write_byte(REG_CTRL6_C, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL7_G);
    // オフセットを有効
    reg[BIT_CTRL7_G_USR_OFF_ON_OUT] = true;
    // CTRL7_Gを有効
    write_byte(REG_CTRL7_G, static_cast<uint8_t>(reg.to_ulong()));

    // 角速度計の設定
    reg = read_byte(REG_CTRL2_G);
    // 出力レートを1.66kHzに設定
    reg[BIT_CTRL2_G_ODR_G3] = true;
    reg[BIT_CTRL2_G_ODR_G2] = false;
    reg[BIT_CTRL2_G_ODR_G1] = false;
    reg[BIT_CTRL2_G_ODR_G0] = false;
    // スケールを+-2000dpsに設定
    reg[BIT_CTRL2_G_FS1_G] = true;
    reg[BIT_CTRL2_G_FS0_G] = true;
    reg[BIT_CTRL2_G_FS_125] = false;
    reg[BIT_CTRL2_G_FS_4000] = false;
    // CTRL2_Gを反映
    write_byte(REG_CTRL2_G, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL4_C);
    // LPF1を有効
    reg[BIT_CTRL4_C_LPF1_SEL_G] = true;
    // CTRL4_Cを反映
    write_byte(REG_CTRL4_C, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL6_C);
    reg[BIT_CTRL6_C_FTYPE_2] = false;
    reg[BIT_CTRL6_C_FTYPE_1] = true;
    reg[BIT_CTRL6_C_FTYPE_0] = false;
    // CTRL6_Cを反映
    write_byte(REG_CTRL6_C, static_cast<uint8_t>(reg.to_ulong()));

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

  const RawAxis &raw_angular_rate() {
    auto res = reinterpret_cast<int16_t *>(rx_buffer_);
    gyro_.x = res[0];
    gyro_.y = res[1];
    gyro_.z = res[2];
    return gyro_;
  }

  const RawAxis &raw_linear_acceleration() {
    auto res = reinterpret_cast<int16_t *>(rx_buffer_);
    accel_.x = res[3];
    accel_.y = res[4];
    accel_.z = res[5];
    return accel_;
  }

  int16_t calibration() {
    // ジャイロは無理?
    int16_t expect_accel[] = {0, 0, 1};

    while (true) {

    }

    return true;
  }
};

Imu::Imu(peripherals::Spi &spi, gpio_num_t spics_io_num)
    : impl_(new Lsm6dsrxImpl(spi, spics_io_num)) {}
Imu::~Imu() = default;

bool Imu::update() { return impl_->update(); }
const Imu::RawAxis &Imu::raw_angular_rate() {
  return impl_->raw_angular_rate();
}
const Imu::RawAxis &Imu::raw_linear_acceleration() {
  return impl_->raw_linear_acceleration();
}
}  // namespace driver::hardware
