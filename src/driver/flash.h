#pragma once

namespace driver::flash
{
  void init();
  bool mounted();

  void ls(const char *const path);
}
