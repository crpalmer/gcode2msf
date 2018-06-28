#include <float.h>
#include "bb.h"

void
bb_init(bb_t *bb)
{
    bb->x[0] = bb->y[0] = DBL_MAX;
    bb->x[1] = bb->y[1] = -DBL_MAX;
}

void
bb_add_point(bb_t *bb, double x, double y)
{
    if (x < bb->x[0]) bb->x[0] = x;
    if (x > bb->x[1]) bb->x[1] = x;
    if (y < bb->y[0]) bb->y[0] = y;
    if (y > bb->y[1]) bb->y[1] = y;
}

void
bb_add_bb(bb_t *dest, const bb_t *other)
{
    bb_add_point(dest, other->x[0], other->y[0]);
    bb_add_point(dest, other->x[1], other->y[1]);
}
