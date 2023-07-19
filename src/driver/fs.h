#pragma once

namespace driver::fs
{
  void init();
  bool mounted();

  void df();
  void ls(const char *const path);
}
