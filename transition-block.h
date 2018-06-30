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
    double mm;
    double density;
} layer_t;

typedef struct {
    int num;
    int from, to;
    double mm;
    double extra_mm;
    int ping;
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

void transition_block_size(double xy[2]);

void transition_block_create_from_runs();

/* temporarily export these two until I get the gcode produced with proper splice information */
double get_pre_transition_mm(int j);
double get_post_transition_mm(int j);


#endif
