#pragma once

namespace driver::fs
{
  void init();
  bool mounted();

  void df();
  void ls(const char *const path);
  void rm(const char *const path);
  void cat(const char *const path);
}
