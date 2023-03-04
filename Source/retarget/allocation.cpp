#include <stdio.h>
#include <stdlib.h>

#include <FreeRTOS.h>

// Override memory management operators

void* operator new(size_t n)
{
    return pvPortMalloc(n);
}

void operator delete(void * p)
{
    vPortFree(p);
}

void operator delete(void * p, size_t n)
{
    vPortFree(p);
}
