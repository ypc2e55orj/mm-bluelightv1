idf_component_register(
  SRCS
  "main.cpp"
  "../src/driver/encoder.cpp"
  "../src/driver/imu.cpp"
  "../src/third-party/lsm6dsrx-pid/lsm6dsrx_reg.c"
  "../src/driver/speaker.cpp"
  INCLUDE_DIRS
  "."
  "../src/"
  "../src/driver"
  "../src/third-party/lsm6dsrx-pid")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
