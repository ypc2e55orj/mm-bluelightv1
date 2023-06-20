idf_component_register(
  SRCS
  "main.cpp"
  "../src/driver/encoder.cpp"
  "../src/driver/imu.cpp"
  "../src/driver/speaker.cpp"
  "../src/third-party/led_strip/led_strip_encoder.c"
  "../src/third-party/lsm6dsrx-pid/lsm6dsrx_reg.c"
  INCLUDE_DIRS
  "."
  "../src/"
  "../src/driver"
  "../src/third-party/led_strip"
  "../src/third-party/lsm6dsrx-pid")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
