#pragma once

#include <stm3210e_eval.h>

#define SHELL_LOG(...) ShellPrintf(##__VA_ARGS__);

void ShellInit();

int ShellPrintf(const char* format, ...);
