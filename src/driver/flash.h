#pragma once

namespace driver::flash
{
  void init();
  bool mounted();

  void df();
  void ls(const char *const path);
}
