#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "printer.h"
#include "transition-block.h"

extern printer_t *printer;

layer_t layers[MAX_RUNS];
int n_layers;
transition_t transitions[MAX_RUNS];
int n_transitions = 0;

/* pre_transition is the amount of filament to consume for the transition prior to printing anything
 * post_transition is the amount of filament to consume for the transition after printing everything
 */

static transition_t *
get_pre_transition(int j)
{
    if (runs[j].pre_transition >= 0) return &transitions[runs[j].pre_transition];
    else return NULL;
}

double
get_pre_transition_mm(int j)
{
    transition_t *t = get_pre_transition(j);
    return t ? printer->transition_target * t->mm : 0;
}

static transition_t *
get_post_transition(int j)
{
    if (runs[j].post_transition >= 0) return &transitions[runs[j].post_transition];
    else return NULL;
}

double
get_post_transition_mm(int j)
{
    transition_t *t = get_post_transition(j);
    return t ? (1-printer->transition_target) * t->mm : 0;
}

static double
transition_block_layer_area(int layer)
{
    layer_t *l = &layers[layer];
    return M_PI*printer->filament/2.0*printer->filament/2.0*l->mm / l->h;
}

static double
transition_block_area()
{
    int i;
    double area = 0;

    for (i = 0; i < n_layers; i++) {
	double la = transition_block_layer_area(i);
	if (la > area) area = la;
    }
    return area;
}

void
transition_block_size(double xy[2])
{
    /* Make y = 2*x (or visa versa) then you get
       area = x * 2*x = 2*x^2
       x = sqrt(area) / sqrt(2)
     */
    double area = transition_block_area();
    double x = sqrt(area) / sqrt(2);
    xy[0] = x;
    xy[1] = 2*x;
}

static double
transition_length(int from, int to, double total_mm)
{
    if (from == to) return 17.7;	// TODO: Can I make this smaller unless we are doing a ping?
    else if (total_mm < 5000) return printer->early_transition_len;
    else return printer->transition_len;
}

static void
add_transition(int from, int to, double z, run_t *run, run_t *pre_run, double *total_mm)
{
    layer_t *layer;

    if (n_layers == 0 || z > layers[n_layers-1].z) {
	layer = &layers[n_layers];
	bb_init(&layer->bb);
	layer->z = z;
	layer->h = z - (n_layers ? layers[n_layers-1].z : 0);
	layer->transition0 = n_transitions;
	layer->n_transitions = 0;
	layer->mm = 0;
	n_layers++;
    }

    pre_run->post_transition = n_transitions;
    run->pre_transition = n_transitions;
    transitions[n_transitions].from = from;
    transitions[n_transitions].to = to;
    transitions[n_transitions].mm = transition_length(from, to, *total_mm);
    n_transitions++;

    layer = &layers[n_layers-1];
    layer->n_transitions++;
    layer->mm += transitions[n_transitions-1].mm;
    bb_add_bb(&layer->bb, &run->bb);

    *total_mm += transitions[n_transitions].mm;
}

static void
compute_transition_tower()
{
    int i;
    double total_mm = 0;

    for (i = 1; i < n_runs; i++) {
	if (runs[i-1].t != runs[i].t) {
	    add_transition(runs[i-1].t, runs[i].t, runs[i].z, &runs[i], &runs[i-1], &total_mm);
	} else if (runs[i-1].z != runs[i].z && (n_layers == 0 || layers[n_layers-1].z != runs[i-1].z)) {
	    add_transition(runs[i-1].t, runs[i-1].t, runs[i-1].z, &runs[i-1], &runs[i-1], &total_mm);
	}
	total_mm += runs[i].e;
    }
}

static void
prune_transition_tower()
{
    int i;
    int last_transition;

    while (n_layers > 0) {
	transition_t *t = &transitions[layers[n_layers-1].transition0];

	if (layers[n_layers-1].n_transitions > 1 || t->from != t->to) break;
	n_layers--;
    }
    n_transitions = layers[n_layers-1].transition0 + layers[n_layers-1].n_transitions;
    for (i = 0; i < n_runs; i++) {
	if (runs[i].pre_transition > n_transitions) runs[i].pre_transition = -1;
	if (runs[i].post_transition > n_transitions) runs[i].post_transition = -1;
    }
}

static void
find_model_bb_to_tower_height(bb_t *model_bb)
{
    int i;

    bb_init(model_bb);

    for (i = 0; i < n_layers; i++) {
        bb_add_bb(model_bb, &layers[i].bb);
    }
}


void
transition_block_create_from_runs(bb_t *model_bb)
{
    compute_transition_tower();
    prune_transition_tower();
    find_model_bb_to_tower_height(model_bb);
}