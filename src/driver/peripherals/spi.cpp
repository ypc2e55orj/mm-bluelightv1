#include "spi.hpp"

// C++
#include <functional>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>

namespace driver::peripherals
{
  class Spi::SpiImpl
  {
  private:
    struct SpiDevice
    {
      spi_device_handle_t handle = nullptr;
      gpio_num_t spics_io_num = GPIO_NUM_NC;
      spi_transaction_t *transaction = nullptr;
      SpiCallback pre_cb;
      SpiCallback post_cb;
    };

    spi_host_device_t host_id_;
    std::vector<SpiDevice *> devices_;

    // 通信前に呼び出されるISR
    static void pre_cb(spi_transaction_t *trans)
    {
      // ESP32の割り込みではfloatを使ってはいけない
      reinterpret_cast<SpiDevice *>(trans->user)->pre_cb(trans);
    }
    // 通信完了時に呼び出されるISR
    static void post_cb(spi_transaction_t *trans)
    {
      // ESP32の割り込みではfloatを使ってはいけない
      reinterpret_cast<SpiDevice *>(trans->user)->post_cb(trans);
    }

  public:
    explicit SpiImpl(spi_host_device_t host_id, gpio_num_t miso_io_num, gpio_num_t mosi_io_num, gpio_num_t sclk_io_num,
                     int max_transfer_sz)
      : host_id_(host_id), devices_()
    {
      spi_bus_config_t bus_config = {};
      bus_config.mosi_io_num = mosi_io_num;
      bus_config.miso_io_num = miso_io_num;
      bus_config.sclk_io_num = sclk_io_num;
      bus_config.max_transfer_sz = max_transfer_sz;
      bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
      bus_config.intr_flags = ESP_INTR_FLAG_IRAM;

      ESP_ERROR_CHECK(spi_bus_initialize(host_id, &bus_config, SPI_DMA_CH_AUTO));
    }
    ~SpiImpl()
    {
      ESP_ERROR_CHECK(spi_bus_free(host_id_));
      for (const auto device : devices_)
      {
        free(device->transaction);
        delete device;
      }
    }

    int add(uint8_t mode, int clock_speed_hz, gpio_num_t spics_io_num, int queue_size, SpiCallback &&pre_cb,
            SpiCallback &&post_cb)
    {
      auto device = new SpiDevice();
      device->spics_io_num = spics_io_num;
      device->pre_cb = std::forward<SpiCallback>(pre_cb);
      device->post_cb = std::forward<SpiCallback>(post_cb);
      device->transaction = (spi_transaction_t *)heap_caps_calloc(1, sizeof(spi_transaction_t), MALLOC_CAP_DMA);
      device->transaction->user = device;

      spi_device_interface_config_t device_interface_config = {};
      device_interface_config.mode = mode;
      device_interface_config.clock_speed_hz = clock_speed_hz;
      device_interface_config.spics_io_num = spics_io_num;
      device_interface_config.queue_size = queue_size;
      device_interface_config.pre_cb = SpiImpl::pre_cb;
      device_interface_config.post_cb = SpiImpl::post_cb;

      if (spi_bus_add_device(host_id_, &device_interface_config, &device->handle) != ESP_OK)
      {
        delete device;
        return -1;
      }
      else
      {
        devices_.push_back(device);
        return static_cast<int>(devices_.size() - 1);
      }
    }

    void update(int index, SpiCallback &&pre_cb, SpiCallback &&post_cb)
    {
      auto device = devices_[index];
      device->pre_cb = std::forward<SpiCallback>(pre_cb);
      device->post_cb = std::forward<SpiCallback>(post_cb);
    }

    bool transaction(int index, TickType_t ticks_to_wait)
    {
      spi_transaction_t *transaction;
      auto device = devices_[index];

      return spi_device_queue_trans(device->handle, device->transaction, portMAX_DELAY) == ESP_OK &&
             spi_device_get_trans_result(device->handle, &transaction, ticks_to_wait) == ESP_OK;
    }

    bool transaction(int index, SpiCallback &&pre_cb, SpiCallback &&post_cb, TickType_t ticks_to_wait)
    {
      auto device = devices_[index];
      device->pre_cb = std::forward<SpiCallback>(pre_cb);
      device->post_cb = std::forward<SpiCallback>(post_cb);

      return transaction(index, ticks_to_wait);
    }
  };

  Spi::Spi(spi_host_device_t host_id, gpio_num_t miso_io_num, gpio_num_t mosi_io_num, gpio_num_t sclk_io_num,
           int max_transfer_sz)
    : impl_(new SpiImpl(host_id, miso_io_num, mosi_io_num, sclk_io_num, max_transfer_sz))
  {
  }
  Spi::~Spi() = default;

  int Spi::add(uint8_t mode, int clock_speed_hz, gpio_num_t spics_io_num, int queue_size, SpiCallback &&pre_cb,
               SpiCallback &&post_cb)
  {
    return impl_->add(mode, clock_speed_hz, spics_io_num, queue_size, std::forward<SpiCallback>(pre_cb),
                      std::forward<SpiCallback>(post_cb));
  }

  bool Spi::transaction(int index, SpiCallback &&pre_cb, SpiCallback &&post_cb, TickType_t ticks_to_wait)
  {
    return impl_->transaction(index, std::forward<SpiCallback>(pre_cb), std::forward<SpiCallback>(post_cb),
                              ticks_to_wait);
  }

  bool Spi::transaction(int index, TickType_t ticks_to_wait)
  {
    return impl_->transaction(index, ticks_to_wait);
  }
}