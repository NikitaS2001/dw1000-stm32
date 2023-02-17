#ifndef _ROSMATH_H_
#define _ROSMATH_H_

#include <math.h>

static __inline double ros_round(double val)
{
    return (val> 0.0) ? floor(val+ 0.5) : ceil(val- 0.5);
}

#endif
