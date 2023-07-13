/**
 * 参考:
 * https://github.com/espressif/esp-idf/blob/release/v5.1/examples/storage/wear_levelling/main/wear_levelling_example_main.c
 */
#include "flash.h"

#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_system.h>

namespace driver::flash
{
  static const auto MOUNT_PATH = "/spiflash";

  static bool mounted = false;
  static wl_handle_t wl = WL_INVALID_HANDLE;

  void mount()
  {
    esp_vfs_fat_mount_config_t mount_cfg = {};
    mount_cfg.max_files = 4;
    mount_cfg.format_if_mount_failed = true;
    mount_cfg.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;

    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_cfg, &wl));
    mounted = true;
  }

  void umount()
  {
    if (!mounted)
    {
      return;
    }

    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(MOUNT_PATH, wl));
  }
}
