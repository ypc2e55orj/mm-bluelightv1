#include "shell.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/uart.h>

#include "../driver/fs.hpp"
#include "../third-party/ntshell/src/lib/core/ntshell.h"
#include "../third-party/ntshell/src/lib/util/ntopt.h"

#include <cstring>

namespace ui::shell
{
  static TaskHandle_t taskHandle = nullptr;

  static int df(int, char **)
  {
    driver::fs::df();
    return 0;
  }
  static int ls(int argc, char **argv)
  {
    if (argc != 2)
    {
      return 1;
    }

    return driver::fs::ls(argv[1]);
  }
  static int rm(int argc, char **argv)
  {
    if (argc != 2)
    {
      return 1;
    }

    return driver::fs::rm(argv[1]);
  }
  static int cat(int argc, char **argv)
  {
    if (argc != 2)
    {
      return 1;
    }

    return driver::fs::cat(argv[1]);
  }
  static int quit(int, char **)
  {
    printf("exiting...\r\n");

    vTaskDelete(nullptr);

    return 0;
  }

  static struct
  {
    const char * c_str;
    int (*func)(int argc, char **argv);
  } commands[] = {
    {"df", df},
    {"ls", ls},
    {"rm", rm},
    {"cat", cat},
    {"quit", quit},
  };

  static int serial_read(char *buf, int count, void *)
  {
    for (int i = 0; i < count; i++)
    {
      while (uart_rx_one_char(reinterpret_cast<uint8_t *>(&buf[i])) != ESP_OK)
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    return count;
  }
  static int serial_write(const char *buf, int count, void *)
  {
    for (int i = 0; i < count; i++)
    {
      while (uart_tx_one_char(static_cast<uint8_t>(buf[i])) != ESP_OK)
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    return count;
  }

  static int ntopt_callback(int argc, char **argv, void *)
  {
    if (argc == 0)
    {
      return 0;
    }
    for (auto & command : commands)
    {
      if (strcmp((const char *)argv[0], command.c_str) == 0)
      {
        return command.func(argc, argv);
      }
    }
    printf("command not found: %s\r\n", argv[0]);

    return 0;
  }
  static int shell_callback(const char *text, void *ext)
  {
    return ntopt_parse(text, ntopt_callback, ext);
  }

  void shellTask(void *)
  {
    ntshell_t nts = {};

    ntshell_init(&nts, serial_read, serial_write, shell_callback, nullptr);
    ntshell_set_prompt(&nts, "-> ");
    ntshell_execute(&nts);
  }

  void start()
  {
    xTaskCreatePinnedToCore(shellTask, "shellTask", 8192, nullptr, 10, &taskHandle, 1);
  }

  void stop()
  {
    vTaskDelete(taskHandle);
  }
}
