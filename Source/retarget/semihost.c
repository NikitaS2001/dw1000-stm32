#include <stdio.h>
#include <stdlib.h>

#include <rt_misc.h>
#include <rt_sys.h>

// Disable semihosting and resolve definitions.
// Otherwise the application doesn't boot without MicroLIB

#pragma import(__use_no_semihosting_swi)
#pragma import(__use_no_semihosting)

struct __FILE { int handle;} ;

FILE __stdout;
FILE __stdin;
FILE __stderr;

int stderr_putchar (int ch) {
  return -1;
}

int stdin_getchar (void) {
  return -1;
}

void _ttywrch (int ch) {
}

void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
