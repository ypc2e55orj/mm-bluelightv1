#pragma once

namespace driver::fs
{
  void init();
  bool mounted();

  void df();
  int ls(char *path);
  int rm(char *path);
  int cat(char *path);
}
