#ifndef __BB_H__
#define __BB_H__

typedef struct {
    double x[2];
    double y[2];
} bb_t;

#define BB_PRINTF_ARGS(bb) (bb)->x[0], (bb)->y[0], (bb)->x[1], (bb)->y[1]

void bb_init(bb_t *);
void bb_add_point(bb_t *, double x, double y);
void bb_add_bb(bb_t *dest, const bb_t *other);

#endif
