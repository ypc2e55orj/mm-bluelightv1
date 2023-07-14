#pragma once

#include <stdio.h>

#define SCI_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
