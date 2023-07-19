#include "flash.h"

#include <esp_vfs.h>
#include <esp_spiffs.h>

#include <string>
#include <cstring>
#include <dirent.h>

namespace driver::flash
{
  static const char *const BASE_PATH = "/spiffs";
  static const char *const PARTITION_LABEL = "storage";

  void init()
  {
    if (mounted())
    {
      return;
    }

    esp_vfs_spiffs_conf_t spiffs_cfg = {};
    spiffs_cfg.base_path = BASE_PATH;
    spiffs_cfg.partition_label = PARTITION_LABEL;
    spiffs_cfg.max_files = 5;
    spiffs_cfg.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_cfg));
  }

  bool mounted()
  {
    return esp_spiffs_mounted(PARTITION_LABEL);
  }

  void df()
  {
    size_t total = 0, used = 0;
    esp_spiffs_info(PARTITION_LABEL, &total, &used);
    printf("%-10s %-10s %-10s %-255s\r", "Size", "Used", "Avail", "Mounted on");
    printf("%-10d %-10d %-10d %-255s\r", total, used, total - used, BASE_PATH);
  }

  void ls(const char *const path)
  {
    static char buffer[256] = {};
    strcpy(buffer, BASE_PATH);
    strcat(buffer, path);

    DIR *dir = opendir(buffer);
    if (dir == nullptr)
    {
      return;
    }

    int index = 0;
    struct dirent *ent = nullptr;
    while ((ent = readdir(dir)) != nullptr)
    {
      printf("%-4d %-255s\r", index++, ent->d_name);
    }

    closedir(dir);
  }
}
