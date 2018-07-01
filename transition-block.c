#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "printer.h"
#include "materials.h"
#include "transition-block.h"

layer_t layers[MAX_RUNS];
int n_layers;
transition_t transitions[MAX_RUNS];
int n_transitions = 0;
transition_block_t transition_block;

#define MIN_FIRST_SPLICE_LEN	140
#define MIN_SPLICE_LEN		 80
#define MIN_PING_LEN		 22	/* Really 20, but give a little slack for pinging off tower */
#define DENSITY_FOR_PERIMETER	0.2

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
    return t ? (1-printer->transition_target) * t->mm + t->extra_mm: 0;
}

static double
transition_block_layer_area(int layer)
{
    layer_t *l = &layers[layer];
    return filament_length_to_mm3(l->mm) / l->h;
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
    double area = transition_block_area();
    double sqrt_area = sqrt(area);
    double ratio = sqrt((1 + sqrt(5))/2);	// Golden ratio
    xy[0] = sqrt_area / ratio;
    xy[1] = sqrt_area * ratio;
    assert(fabs(xy[0] * xy[1] - area) < 0.0001);
}

static double
transition_length(int from, int to, double total_mm)
{
    double mn = printer->min_transition_len;
    double mx = total_mm < 5000 ? printer->early_transition_len : printer->transition_len;
    active_material_t *in = get_active_material(to);
    active_material_t *out = get_active_material(from);
    double factor;

    if (from == to) return 0;

    if (in->strength == STRONG) {
	factor = 0;
    } else if (in->strength == WEAK) {
	if (out->strength == WEAK) factor = 0.5;
	else if (out->strength == STRONG) factor = 1;
	else factor = 0.75;
    } else {
	if (out->strength == STRONG) factor = 0.75;
	else if (out->strength == WEAK) factor = 0.25;
	else factor = 0.5;
    }

    return mn + (mx - mn) * factor;
}

static void
add_transition(int from, int to, double z, run_t *run, run_t *pre_run, double *total_mm, double *filament_mm)
{
    layer_t *layer;
    double mm;

    if (n_layers == 0 || z > layers[n_layers-1].z) {
	layer = &layers[n_layers];
	layer->num = n_layers;
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
    transitions[n_transitions].num = n_transitions;
    transitions[n_transitions].from = from;
    transitions[n_transitions].to = to;
    transitions[n_transitions].mm = transition_length(from, to, *total_mm);
    transitions[n_transitions].ping = 0;

    (*filament_mm) += printer->transition_target * transitions[n_transitions].mm;
    if (n_transitions == 0 && *filament_mm < MIN_FIRST_SPLICE_LEN) {
	transitions[n_transitions].extra_mm = MIN_FIRST_SPLICE_LEN - *filament_mm;
    } else if (from == to) {
	transitions[n_transitions].extra_mm = 0;
    } else if (*filament_mm < MIN_SPLICE_LEN) {
	transitions[n_transitions].extra_mm = MIN_SPLICE_LEN - *filament_mm;
    } else {
	transitions[n_transitions].extra_mm = 0;
    }

    mm = transitions[n_transitions].mm + transitions[n_transitions].extra_mm;

    if (from != to) (*filament_mm) = (1 - printer->transition_target) * transitions[n_transitions].mm;

    n_transitions++;

    layer = &layers[n_layers-1];
    layer->n_transitions++;

    layer->mm += mm;
    (*total_mm) += mm;

    bb_add_bb(&layer->bb, &pre_run->bb);
    bb_add_bb(&layer->bb, &run->bb);
}

#define PING_THRESHOLD 425

static void
compute_transition_tower()
{
    int i;
    double total_mm, filament_mm;
    double last_ping_at = 0;

    total_mm = filament_mm = runs[0].e;

    for (i = 1; i < n_runs; i++) {
	double ping_delta;

	if (runs[i-1].t != runs[i].t) {
	    add_transition(runs[i-1].t, runs[i].t, runs[i].z, &runs[i], &runs[i-1], &total_mm, &filament_mm);
	} else if (runs[i-1].z != runs[i].z && (n_layers == 0 || layers[n_layers-1].z != runs[i-1].z)) {
	    add_transition(runs[i-1].t, runs[i-1].t, runs[i-1].z, &runs[i-1], &runs[i-1], &total_mm, &filament_mm);
	}

	ping_delta = total_mm - last_ping_at;
        if (ping_delta > PING_THRESHOLD || (i+1 < n_runs && ping_delta+runs[i+1].e > PING_THRESHOLD*2)) {
	    last_ping_at = total_mm;
	    transitions[n_transitions-1].ping = 1;
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
static double
layer_min_density(int i)
{
    return (i == 0 ? printer->min_bottom_density : printer->min_density);
}

static void
add_min_transition_lengths(double area)
{
    int i;

    for (i = 0; i < n_layers; i++) {
	if (layers[i].n_transitions == 1) {
	    double this_area = area*layer_min_density(i);
	    double mm3 = this_area * layers[i].h;
	    double len = filament_mm3_to_length(mm3);
	    int has_ping =  transitions[layers[i].transition0].ping;
	    double len_with_ping = has_ping && len < MIN_PING_LEN ? MIN_PING_LEN : len;
	    transition_t *t = &transitions[layers[i].transition0];
	    double need_mm = len_with_ping - (t->mm + t->extra_mm);

	    if (need_mm > 0) {
		layers[i].mm += need_mm;
		t->mm += need_mm;
	    }
	}
    }
}

static void
reduce_pings()
{
    int i, j;

    for (i = 0; i < n_transitions; i++) {
	if (transitions[i].ping) {
	    transitions[i].ping = 0;
	    for (j = i; j < n_transitions && transitions[j].from == transitions[j].to; j++) {
		transitions[j].ping = 0;
	    }
	    transitions[j-1].ping = 1;
	}
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

#define BLOCK_SEP 5

static void
place_transition_block_common(bb_t *model_bb, double *d, double mid[2])
{
    double size[2];
    int mn = 0;
    int i;

    transition_block_size(size);

    for (i = 1; i < 4; i++) if (d[i] < d[mn]) mn = i;

    switch(mn) {
    case 0:
	transition_block.x = model_bb->x[0] - BLOCK_SEP - size[0];
	transition_block.y = mid[1]-size[1]/2;
	transition_block.w = size[0];
	transition_block.h = size[1];
	break;
    case 1:
	transition_block.x = model_bb->x[1] + BLOCK_SEP;
	transition_block.y = mid[1]-size[1]/2;
	transition_block.w = size[0];
	transition_block.h = size[1];
	break;
    case 2:
	transition_block.x = mid[0]-size[1]/2;
	transition_block.y = model_bb->y[0] - BLOCK_SEP - size[0];
	transition_block.w = size[1];
	transition_block.h = size[0];
	break;
    case 3:
	transition_block.x = mid[0]-size[1]/2;
	transition_block.y = model_bb->y[1] + BLOCK_SEP;
	transition_block.w = size[1];
	transition_block.h = size[0];
	break;
    }
}

static void
place_transition_block_delta()
{
    bb_t model_bb;
    double d[4];
    double mid[2] = { 0, 0 };

    find_model_bb_to_tower_height(&model_bb);

    d[0] = -model_bb.x[0];
    d[1] = model_bb.x[1];
    d[2] = -model_bb.y[0];
    d[3] = model_bb.y[1];

    place_transition_block_common(&model_bb, d, mid);
}

static void
place_transition_block_cartesian()
{
    bb_t model_bb;
    double d[4];
    double mid[2] = { printer->bed_x / 2, printer->bed_y / 2 };

    find_model_bb_to_tower_height(&model_bb);

    d[0] = model_bb.x[0];
    d[1] = printer->bed_x - model_bb.x[1];
    d[2] = model_bb.y[0];
    d[3] = printer->bed_y - model_bb.y[1];

    place_transition_block_common(&model_bb, d, mid);
}

static void
place_transition_block()
{
    if (printer->circular) place_transition_block_delta();
    else place_transition_block_cartesian();
    transition_block.area = transition_block_area();
}

static void
compute_densities_and_perimeters()
{
    int i;

    for (i = 0; i < n_layers; i++) {
	double area = filament_length_to_mm3(layers[i].mm) / layers[i].h;
	double total_area = transition_block.area;
	double min_density = layer_min_density(i);

	layers[i].density = area / total_area;
	layers[i].use_perimeter = (i == 0 || layers[i].density <= DENSITY_FOR_PERIMETER);

	if (layers[i].use_perimeter) {
	    double perimeter_area = 2*(transition_block.w + transition_block.h)*printer->nozzle*layers[i].h;
	    area -= perimeter_area;
	    total_area -= perimeter_area;
	    layers[i].density = area / total_area;
	}

	if (layers[i].density < min_density) {
	    double needed = filament_mm3_to_length(min_density*total_area*layers[i].h) - filament_mm3_to_length(layers[i].density*total_area*layers[i].h);
	    transitions[layers[i].transition0].mm += needed;
	    layers[i].mm += needed;
	    layers[i].density = min_density;
	}

	assert(layers[i].density <= 1);
    }
}

void
transition_block_create_from_runs()
{
    compute_transition_tower();
    prune_transition_tower();
    add_min_transition_lengths(transition_block_area());
    reduce_pings();
    place_transition_block();
    compute_densities_and_perimeters();
}
