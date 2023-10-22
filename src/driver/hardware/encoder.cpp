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
    static constexpr float RESOLUTION_PER_DEGREE = 360.f / static_cast<float>(RESOLUTION);

    // 送信データにパリティを付与
    static constexpr uint16_t REG_MASTER_RESET = 0x33A5;
    static constexpr uint16_t REG_ANGULAR_DATA = 0x3FFF;

    static constexpr uint16_t command_frame(uint16_t reg, bool is_reading)
    {
      reg = (reg << 1) | (is_reading ? 0x8000 : 0x0000);
      return (reg | __builtin_parity(reg));
    }
    static bool verify_frame(uint16_t res)
    {
      bool parity_ok = (res & 0x0001) == __builtin_parity(res >> 1);
      bool command_ok = (res & 0x0002) != 0x0002;
      return parity_ok && command_ok;
    }

  public:
    explicit AS5050AImpl(peripherals::Spi &spi, gpio_num_t spics_io_num) : spi_(spi), angle_(0)
    {
      // デバイスを追加
      index_ = spi_.add(
        0, 0, 1, SPI_MASTER_FREQ_10M, spics_io_num, 1, [](spi_transaction_t *) {}, [](spi_transaction_t *) {});
      auto trans = spi_.transaction(index_);
      // リセット
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->tx_data[0] = command_frame(REG_MASTER_RESET, false) >> 8;
      trans->tx_data[1] = command_frame(REG_MASTER_RESET, false) & 0xFF;
      trans->length = 16;
      spi_.transmit(index_);

      // 角度を取得
      trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      trans->tx_data[0] = command_frame(REG_ANGULAR_DATA, true) >> 8;
      trans->tx_data[1] = command_frame(REG_ANGULAR_DATA, true) & 0xFF;
      trans->length = 16;
      spi_.transmit(
        index_, [](spi_transaction_t *) {},
        [&](spi_transaction_t *trans) {
          uint16_t res = trans->rx_data[0] << 8 | trans->rx_data[1];
          if (verify_frame(res))
            angle_ = res >> 2 & 0x3FF;
        });
    }
    ~AS5050AImpl() = default;

    bool update() override
    {
      return spi_.transmit(index_);
    }

    [[nodiscard]] float radian() const
    {
      return static_cast<float>(angle_) * RESOLUTION_PER_RADIAN;
    }

    [[nodiscard]] float degree() const
    {
      return static_cast<float>(angle_) * RESOLUTION_PER_DEGREE;
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
  float Encoder::degree()
  {
    return impl_->degree();
  }
}
