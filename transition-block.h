#ifndef __TRANSITION_BLOCK_H__
#define __TRANSITION_BLOCK_H__

#include "gcode.h"

typedef struct {
    int    num;
    bb_t   bb;
    double z;
    double h;
    int transition0;
    int n_transitions;
    double density;
    int    use_perimeter;
} layer_t;

typedef struct {
    int num;
    int from, to;
    long offset;
    double mm_from_runs;
    double mm_pre_transition;
    double infill_mm;
    double pre_mm, post_mm;
    double support_mm;
    double avail_infill;
    double avail_support;
    int ping;
    int next_move_no_extrusion;
} transition_t;

typedef struct {
    double x, y, w, h;
    double area;
} transition_block_t;

extern layer_t layers[MAX_RUNS];
extern int n_layers;
extern transition_t transitions[MAX_RUNS];
extern int n_transitions;
extern transition_block_t transition_block;
extern double transition_final_mm;
extern double transition_final_waste;

extern int reduce_pings;

void transition_block_size(double xy[2]);

void transition_block_create_from_runs();

void transition_block_dump_transitions(FILE *o);

#endif
