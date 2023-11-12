#include "fs.hpp"

// C++

// ESP-IDF
#include <esp_spiffs.h>

namespace driver::hardware {
class Fs::FsImpl {
 private:
  static constexpr auto BASE_PATH = "/spiffs";
  static constexpr auto PARTITION_LABEL = "storage";

 public:
  explicit FsImpl(size_t max_files) {
    esp_vfs_spiffs_conf_t spiffs_config = {};
    spiffs_config.base_path = BASE_PATH;
    spiffs_config.partition_label = PARTITION_LABEL;
    spiffs_config.max_files = max_files;
    spiffs_config.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_config));
  }

  static void info(size_t &total, size_t &used) {
    ESP_ERROR_CHECK(esp_spiffs_info(PARTITION_LABEL, &total, &used));
  }
};

Fs::Fs(size_t max_files) : impl_(new FsImpl(max_files)) {}
Fs::~Fs() = default;

void Fs::info(size_t &total, size_t &used) { return impl_->info(total, used); }
}  // namespace driver::hardware
