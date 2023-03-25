#pragma once

#include <stm3210e_eval.h>

#define SHELL_LOG(f, ...) ShellPrintf(f, ##__VA_ARGS__);

#ifdef __cplusplus
extern "C"
{
#endif

void ShellInit();

int ShellPrintf(const char* format, ...);

#ifdef __cplusplus
}
#endif
