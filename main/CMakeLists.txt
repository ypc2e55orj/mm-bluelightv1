file(
  GLOB_RECURSE SOURCES
  "main.cpp"
  "../src/*.cpp"
  "../src/*.hpp"
  "../third-party/led_strip/*.c"
  "../third-party/lsm6dsrx-pid/*.c"
  "../third-party/musical_buzzer/*.c"
  "../third-party/ntshell/src/lib/*.c")

message("### mm-bluelight ##")
foreach (SOURCE IN LISTS SOURCES)
    message("Add: ${SOURCE}")
endforeach ()

idf_component_register(
  SRCS
  ${SOURCES}
  INCLUDE_DIRS
  "../src"
  "../third-party"
  "../third-party/led_strip"
  "../third-party/lsm6dsrx-pid"
  "../third-party/musical_buzzer"
  "../third-party/ntshell/src/lib/core"
  "../third-party/ntshell/src/lib/util")

component_compile_options(
  -std=gnu++23
  -Wall
  -Wextra
  -Wdouble-promotion
  -Wfloat-equal
)
