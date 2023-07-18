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

  void info(size_t *total, size_t *used)
  {
    esp_spiffs_info(PARTITION_LABEL, total, used);
  }

  static const std::string d_type_str(uint8_t type)
  {
    switch (type)
    {
    case DT_REG:
      return "reg";
    case DT_DIR:
      return "dir";
    case DT_UNKNOWN:
    default:
      return "unknown";
    }
  }
  void ls(const char *const path)
  {
    static char buffer[512] = {};
    strcpy(buffer, BASE_PATH);
    strcat(buffer, path);

    DIR *dir = opendir(buffer);
    if (dir == nullptr)
    {
      return;
    }

    struct dirent *ent = nullptr;
    while ((ent = readdir(dir)) != nullptr)
    {
      printf("%03u %255s %10s\n", ent->d_ino, ent->d_name, d_type_str(ent->d_type).c_str());
    }

    closedir(dir);
  }
}
