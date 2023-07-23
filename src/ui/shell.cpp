#include "shell.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/uart.h>

#include "../driver/fs.h"
#include "../third-party/ntshell/src/lib/core/ntshell.h"

#include <cstdint>
#include <cstring>

namespace shell
{
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

  static int user_callback(const char *text, void *)
  {
    if (strcmp(text, "df") == 0)
    {
      driver::fs::df();
    }
    else if (strcmp(text, "ls") == 0)
    {
      driver::fs::ls("/spiffs");
    }
    else
    {
      printf("%s\r\n", text);
    }

    return 0;
  }

  void shell()
  {
    ntshell_t nts = {};

    ntshell_init(&nts, serial_read, serial_write, user_callback, nullptr);
    ntshell_set_prompt(&nts, "-> ");
    ntshell_execute(&nts);
  }
}
