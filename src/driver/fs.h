#pragma once

namespace driver::fs
{
  void init();
  bool mounted();

  int df(int argc, char **argv);
  int ls(int argc, char **argv);
  int rm(int argc, char **argv);
  int cat(int argc, char **argv);
}
