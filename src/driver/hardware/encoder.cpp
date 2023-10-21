#include "encoder.hpp"

// C++
#include <numbers>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>

// Project
#include "base.hpp"

namespace driver::hardware
{
  class Encoder::AS5050AImpl final : DriverBase
  {
  private:
    peripherals::Spi &spi_;
    int index_;
    uint16_t angle_;

    // 分解能あたりの角度
    static constexpr uint16_t RESOLUTION = 1024; // 10 bit
    static constexpr float RESOLUTION_PER_RADIAN = (2.0f * std::numbers::pi_v<float>) / static_cast<float>(RESOLUTION);

    // 送信データにパリティを付与
    static constexpr uint16_t REG_MASTER_RESET = 0x33A5;
    static constexpr uint16_t REG_ANGULAR_DATA = 0x3FFF;
    static constexpr uint16_t read(uint16_t reg)
    {
      reg = (reg << 1) | 0x8000;
      return (reg | __builtin_parity(reg));
    }
    static constexpr uint16_t write(uint16_t reg)
    {
      return ((reg << 1) | __builtin_parity(reg));
    }

    // 送信前にデータを準備する
    static void pre_init(spi_transaction_t *trans)
    {
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->length = 16;
      trans->tx_data[0] = write(REG_MASTER_RESET) >> 8;
      trans->tx_data[1] = write(REG_MASTER_RESET) & 0xFF;
    }
    static void pre_get(spi_transaction_t *trans)
    {
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->length = 16;
      trans->tx_data[0] = read(REG_ANGULAR_DATA) >> 8;
      trans->tx_data[1] = read(REG_ANGULAR_DATA) & 0xFF;
    }
    // 受信後にデータを変換
    void post_get(spi_transaction_t *trans)
    {
      angle_ = ((trans->rx_data[0] << 8 | trans->rx_data[1]) & 0x3FFE) >> 2;
    }

  public:
    explicit AS5050AImpl(peripherals::Spi &spi, gpio_num_t spics_io_num) : spi_(spi), angle_(0)
    {
      // デバイスを追加、リセット
      index_ = spi_.add(1, SPI_MASTER_FREQ_10M, spics_io_num, 1, pre_init, [](spi_transaction_t *trans) {});
      // コールバックを更新 (角度を取得)
      spi_.transaction(
        index_, pre_get, [&](spi_transaction_t *trans) { post_get(trans); }, portMAX_DELAY);
    }
    ~AS5050AImpl() = default;

    bool update() override
    {
      return spi_.transaction(index_, portMAX_DELAY);
    }

    [[nodiscard]] float radian() const
    {
      return static_cast<float>(angle_) * RESOLUTION_PER_RADIAN;
    }
  };

  Encoder::Encoder(peripherals::Spi &spi, gpio_num_t spics_io_num) : impl_(new AS5050AImpl(spi, spics_io_num))
  {
  }
  Encoder::~Encoder() = default;

  bool Encoder::update()
  {
    return impl_->update();
  }
  float Encoder::radian()
  {
    return impl_->radian();
  }
}
