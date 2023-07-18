/**
 * 参考:
 * https://github.com/espressif/esp-idf/blob/release/v5.1/examples/storage/wear_levelling/main/wear_levelling_example_main.c
 */
#include "flash.h"

#include <esp_vfs.h>
#include <esp_spiffs.h>

namespace driver::flash
{
  static bool mounted = false;

  void init()
  {
    if (mounted)
    {
      return;
    }

    esp_vfs_spiffs_conf_t spiffs_cfg = {};
    spiffs_cfg.base_path = "/spiffs";
    spiffs_cfg.partition_label = nullptr;
    spiffs_cfg.max_files = 5;
    spiffs_cfg.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_cfg));

    mounted = true;
  }
}
