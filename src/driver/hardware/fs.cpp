#include "fs.hpp"

#include <esp_spiffs.h>
#include <esp_vfs.h>

#include <cstring>
#include <dirent.h>

namespace driver::fs
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
    printf("%-10s %-10s %-10s %-.10s\r\n", "Size", "Used", "Avail", "Mounted on");
    printf("%-10d %-10d %-10d %-.64s\r\n", total, used, total - used, BASE_PATH);
  }

  int ls(char *path)
  {
    DIR *dir = opendir(path);
    if (dir == nullptr)
    {
      return 1;
    }

    int index = 0;
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr)
    {
      printf("%-4d %-.64s\r\n", index++, ent->d_name);
    }

    closedir(dir);

    return 0;
  }

  int rm(char *path)
  {
    return unlink(path);
  }

  int cat(char *path)
  {
    char buffer[256] = {};

    FILE *file = fopen(path, "r");
    if (!file)
    {
      return 1;
    }
    
    while (fgets(buffer, 256, file) != nullptr)
    {
      printf("%s", buffer);
    }
    fclose(file);

    return 0;
  }
}
