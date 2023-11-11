file(
        GLOB_RECURSE SOURCES
        "main.cpp"
        "../src/*.cpp"
        "../src/*.hpp"
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
        "../third-party/ntshell/src/lib/core"
        "../third-party/ntshell/src/lib/util")

component_compile_options(
        -std=gnu++23
        -mtext-section-literals
        -Wall
        -Wextra
        -Wdouble-promotion
        -Wfloat-equal)
