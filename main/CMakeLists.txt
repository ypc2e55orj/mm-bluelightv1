file(
        GLOB_RECURSE SOURCES
        "main.cpp"
        "../src/*.cpp"
        "../src/*.hpp")

message("### mm-bluelight ##")
foreach (SOURCE IN LISTS SOURCES)
    message("Add: ${SOURCE}")
endforeach ()

idf_component_register(
        SRCS
        ${SOURCES}
        INCLUDE_DIRS
        "../src")

component_compile_options(
        -std=gnu++23
        -mtext-section-literals
        -Wall
        -Wextra
        -Wdouble-promotion
        -Wfloat-equal)
