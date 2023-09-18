file(
  GLOB SOURCES
  "main.cpp"
  "../src/*.cpp"
  "../src/*.hpp"
  "../src/data/*.cpp"
  "../src/data/*.hpp"
  "../src/driver/*.cpp"
  "../src/driver/*.hpp"
  "../src/filter/*.cpp"
  "../src/filter/*.hpp"
  "../src/ui/*.cpp"
  "../src/ui/*.hpp"
  "../src/wrapper/*.cpp"
  "../src/wrapper/*.hpp"
  "../src/third-party/led_strip/*.c"
  "../src/third-party/led_strip/*.h"
  "../src/third-party/lsm6dsrx-pid/*.c"
  "../src/third-party/lsm6dsrx-pid/*.h"
  "../src/third-party/musical_buzzer/*.c"
  "../src/third-party/musical_buzzer/*.h"
  "../src/third-party/ntshell/src/lib/core/*.c"
  "../src/third-party/ntshell/src/lib/core/*.h"
  "../src/third-party/ntshell/src/lib/util/*.c"
  "../src/third-party/ntshell/src/lib/util/*.h")

message("### mm-bluelight ##")
message("SOURCE: ${SOURCES}")

idf_component_register(
  SRCS
  ${SOURCES}
  INCLUDE_DIRS
  "."
  "../src/"
  "../src/data"
  "../src/driver"
  "../src/filter"
  "../src/ui"
  "../src/wrapper"
  "../src/third-party/led_strip"
  "../src/third-party/lsm6dsrx-pid"
  "../src/third-party/musical_buzzer"
  "../src/third-party/ntshell/src/lib/core"
  "../src/third-party/ntshell/src/lib/util")

component_compile_options(
  -std=gnu++23
  -Wall
  -Wextra
  -Wdouble-promotion
  -Wfloat-equal
)
