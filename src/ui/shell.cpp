#include "shell.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/uart.h>

#include "../driver/fs.h"
#include "../third-party/ntshell/src/lib/core/ntshell.h"
#include "../third-party/ntshell/src/lib/util/ntopt.h"

#include <cstring>

namespace ui
{
  static struct
  {
    const char * c_str;
    int (*func)(int argc, char **argv);
  } commands[] = {
    {"df", driver::fs::df},
    {"ls", driver::fs::ls},
    {"rm", driver::fs::rm},
    {"cat", driver::fs::cat},
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
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
    {
      if (strcmp((const char *)argv[0], commands[i].c_str) == 0)
      {
        return commands[i].func(argc, argv);
      }
    }
    printf("command not found: %s\r\n", argv[0]);

    return 0;
  }
  static int shell_callback(const char *text, void *ext)
  {
    return ntopt_parse(text, ntopt_callback, ext);
  }

  void shell()
  {
    ntshell_t nts = {};

    ntshell_init(&nts, serial_read, serial_write, shell_callback, nullptr);
    ntshell_set_prompt(&nts, "-> ");
    ntshell_execute(&nts);
  }
}
