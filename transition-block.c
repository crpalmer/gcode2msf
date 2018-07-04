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

#define EXTRA_FILAMENT  75

layer_t layers[MAX_RUNS];
int n_layers;
double layer_mm[MAX_RUNS];
transition_t transitions[MAX_RUNS];
int n_transitions = 0;
transition_block_t transition_block;
int reduce_pings = 0;
double transition_final_mm;
double transition_final_waste;

#define MIN_FIRST_SPLICE_LEN	141	/* There appears to be some epsilon error with 140 causing it to error */
#define MIN_SPLICE_LEN		 80
#define MIN_PING_LEN		 22	/* Really 20, but give a little slack for pinging off tower */
#define DENSITY_FOR_PERIMETER	0.2

static double
layer_transition_mm(layer_t *l)
{
    transition_t *t;
    double mm = 0;
    int i;

    for (t = &transitions[l->transition0], i = 0; i < l->n_transitions; t++, i++) {
	mm += t->pre_mm + t->post_mm;
    }
    return mm;
}

static double
transition_block_layer_area(int layer)
{
    layer_t *l = &layers[layer];
    int i;
    transition_t *t;
    double mm = 0;

    for (t = &transitions[l->transition0], i = 0; i < l->n_transitions; i++, t++) {
	mm += t->pre_mm + t->post_mm;
    }

    return filament_length_to_mm3(mm) / l->h;
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
    double mx = printer->transition_len;
    active_material_t *in = get_active_material(to);
    active_material_t *out = get_active_material(from);
    double factor;

    if (from == to) return 0;
    if (total_mm < 5000) return printer->early_transition_len;

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
move_purge(double *from, double *to, double *avail)
{
    double reduce = *from > *avail ? *avail : *from;
    *from -= reduce;
    *avail -= reduce;
    *to += reduce;
}

static void
add_extra_block_purge(transition_t *t, double extra_mm)
{
    if (extra_mm > 0) {
	move_purge(&t->infill_mm, &t->pre_mm, &extra_mm);
	move_purge(&t->support_mm, &t->post_mm, &extra_mm);
	t->pre_mm += extra_mm/2;
	t->post_mm += extra_mm/2;
    }
}

static void
layer_add_extra_block_purge(layer_t *l, double extra_mm)
{
    int i;
    transition_t *t;

    for (t = &transitions[l->transition0], i = 0; i < l->n_transitions; t++, i++) {
	add_extra_block_purge(t, extra_mm / l->n_transitions);
    }
}

/* Assumption is that as the adaptive feedback system gets cranking,
 * it should stabilize on reasonable values.
 * Give it 5M to get started, 5M to stabilize and then start reducing
 * the number of pings.
 * After long enough, really reduce them to speed up large prints.
 */

#define PING_THRESHOLD 425
#define PING_SECOND_THRESHOLD	(PING_THRESHOLD*2)
#define PING_SECOND_E		10000
#define PING_THIRD_THRESHOLD	(PING_SECOND_THRESHOLD*4)
#define PING_THIRD_E		50000

static double
get_ping_threshold(double total_mm)
{
    if (reduce_pings) {
	if (total_mm >= PING_THIRD_E) return PING_THIRD_THRESHOLD;
	if (total_mm >= PING_SECOND_E) return PING_SECOND_THRESHOLD;
    }
    return PING_THRESHOLD;
}

static void
add_transition(int from, int to, double z, run_t *run, run_t *pre_run, double *mm_from_runs, double *total_mm, double *filament_mm)
{
    transition_t *t;
    layer_t *layer;
    double mm;

    if (n_layers == 0 || z > layers[n_layers-1].z) {
	layer = &layers[n_layers];
	layer->num = n_layers;
	bb_init(&layer->bb);
	layer->z = z;
	layer->h = z - (n_layers ? layers[n_layers-1].z : 0);
	layer->transition0 = n_transitions;
	layer->n_transitions = 1;
	n_layers++;
    } else {
	layers[n_layers-1].n_transitions++;
    }

    pre_run->post_transition = n_transitions;
    run->pre_transition = n_transitions;

    t = &transitions[n_transitions++];

    t->num = n_transitions-1;
    t->from = from;
    t->to = to;
    t->ping = 0;
    t->mm_from_runs = *mm_from_runs;
    t->mm_pre_transition = *filament_mm;
    t->offset = pre_run->offset;

    t->avail_infill = printer->transition_in_infill && pre_run->trailing_infill_mm > 0 ? pre_run->trailing_infill_mm : 0;
    t->avail_support = printer->transition_in_support && run->leading_support_mm > 0 ? run->leading_support_mm : 0;

    mm = transition_length(from, to, *total_mm);
    t->pre_mm = printer->transition_target * mm;
    t->post_mm = mm - t->pre_mm;
    t->infill_mm = t->support_mm = 0;

    move_purge(&t->pre_mm, &t->infill_mm, &t->avail_infill);
    move_purge(&t->post_mm, &t->support_mm, &t->avail_support);
    move_purge(&t->post_mm, &t->infill_mm, &t->avail_infill);
    move_purge(&t->pre_mm, &t->support_mm, &t->avail_support);

    if (from != to) {
	(*mm_from_runs) = 0;
	(*filament_mm) = t->post_mm;
    } else {
	(*filament_mm) += t->pre_mm + t->post_mm;
    }

    (*total_mm) += t->pre_mm + t->post_mm;

    bb_add_bb(&layer->bb, &pre_run->bb);
    bb_add_bb(&layer->bb, &run->bb);
}

static void
compute_transition_tower()
{
    int i;
    double mm_from_runs, total_mm, filament_mm;
    double last_ping_at = 0;
    double ping_threshold;

    mm_from_runs = total_mm = filament_mm = runs[0].e;
    ping_threshold = get_ping_threshold(total_mm);

    for (i = 1; i < n_runs; i++) {
	double ping_delta;

	if (runs[i-1].t != runs[i].t) {
	    add_transition(runs[i-1].t, runs[i].t, runs[i].z, &runs[i], &runs[i-1], &mm_from_runs, &total_mm, &filament_mm);
	} else if (runs[i-1].z != runs[i].z && (n_layers == 0 || layers[n_layers-1].z != runs[i-1].z)) {
	    add_transition(runs[i-1].t, runs[i-1].t, runs[i-1].z, &runs[i-1], &runs[i-1], &mm_from_runs, &total_mm, &filament_mm);
	}

	ping_delta = total_mm - last_ping_at;
        if (ping_delta > ping_threshold || (total_mm > ping_threshold && i+1 < n_runs && ping_delta+runs[i+1].e > ping_threshold*2)) {
	    last_ping_at = total_mm;
	    transitions[n_transitions-1].ping = 1;
	    ping_threshold = get_ping_threshold(total_mm);
	}
	total_mm += runs[i].e;
	filament_mm += runs[i].e;
	mm_from_runs += runs[i].e;
    }
    transition_final_mm = mm_from_runs;
    transition_final_waste = (printer->bowden_len > 0 ? printer->bowden_len : 0) + EXTRA_FILAMENT;
}

static void
prune_transition_tower()
{
    int i;

    while (n_layers > 0 && layer_transition_mm(&layers[n_layers-1]) == 0) n_layers--;

    n_transitions = layers[n_layers-1].transition0 + layers[n_layers-1].n_transitions;
    for (i = 0; i < n_runs; i++) {
	if (runs[i].pre_transition > n_transitions) runs[i].pre_transition = -1;
	if (runs[i].post_transition > n_transitions) runs[i].post_transition = -1;
    }
}

static void
reduce_pings_when_no_splices()
{
    int i, j;

    for (i = 0; i < n_transitions; i++) {
	if (transitions[i].ping && transitions[i].from == transitions[i].to) {
	    transitions[i].ping = 0;
	    for (j = i+1; j < n_transitions && transitions[j].from == transitions[j].to; j++) {
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

static double
layer_min_density(int i)
{
    return (i == 0 ? printer->min_bottom_density : printer->min_density);
}

static double
layer_perimeter_area(layer_t *l)
{
    return l->use_perimeter ? 2*(transition_block.w + transition_block.h)*printer->nozzle : 0;
}

static double
layer_perimeter_filament_len(layer_t *l)
{
    return filament_mm3_to_length(layer_perimeter_area(l)*l->h);
}

static void
compute_layer_density(layer_t *l, double *area_out)
{
    double this_mm = layer_transition_mm(l);
    double this_area = filament_length_to_mm3(this_mm) / l->h;
    double total_area = transition_block.area;
    double perimeter_area = layer_perimeter_area(l);

    l->density = (this_area - perimeter_area)/ (total_area - perimeter_area);

    if (area_out) *area_out = total_area - perimeter_area;
}

static int
fix_constraints()
{
    int i, j;
    layer_t *l;
    int bad = 0;

    for (l = layers, i = 0; i < n_layers; i++, l++) {
	transition_t *t0 = &transitions[l->transition0];
	transition_t *t;
	double min_density = layer_min_density(i);
	double perimeter_len;
	double area;

	compute_layer_density(l, &area);
	if (i == 0 || l->density <= DENSITY_FOR_PERIMETER) {
	    l->use_perimeter = 1;
	    compute_layer_density(l, &area);
	}

	perimeter_len = layer_perimeter_filament_len(l);

	if (l->use_perimeter && t0->pre_mm + t0->post_mm < perimeter_len) {
	    add_extra_block_purge(t0, perimeter_len - (t0->pre_mm + t0->post_mm));
	    compute_layer_density(l, &area);
	}

	for (t = t0, j = 0; j < l->n_transitions; t++, j++) {
	    if (t->num == 0 && t->mm_pre_transition + t->pre_mm < MIN_FIRST_SPLICE_LEN) {
		t->pre_mm += MIN_FIRST_SPLICE_LEN - (t->mm_pre_transition + t->pre_mm);
	    } else if (t->from != t->to && t->mm_pre_transition + t->pre_mm < MIN_SPLICE_LEN) {
		double needed = MIN_SPLICE_LEN - (t->mm_pre_transition + t->pre_mm);
		move_purge(&t->support_mm, &t->pre_mm, &needed);
		t->pre_mm += needed;
	    }
	    if (t->ping && t->pre_mm + t->post_mm < MIN_PING_LEN) {
		add_extra_block_purge(t, MIN_PING_LEN - (t->pre_mm + t->post_mm));
	    }
	    compute_layer_density(l, &area);
	}

	if (l->density < min_density) {
	    double needed = filament_mm3_to_length(area * (min_density - l->density) * l->h);
	    layer_add_extra_block_purge(l, needed);
	    compute_layer_density(l, &area);
	    assert(min_density -  0.01 <= l->density && l->density <= min_density + 0.01);
	}

	bad = bad || l->density > 1.001;
    }

    return ! bad;
}

void
transition_block_create_from_runs()
{
    int iterations = 0;
    compute_transition_tower();
    prune_transition_tower();
    reduce_pings_when_no_splices();
    do {
	iterations++;
	place_transition_block();
    } while (! fix_constraints());
    printf("It took %d iterations to stabilize the block\n", iterations);
}

void
transition_block_dump_transitions(FILE *o)
{
    layer_t *l;
    transition_t *t;
    int i, j;
    double mm[N_DRIVES] = { 0, };
    double infill[N_DRIVES] = { 0, };
    double waste[N_DRIVES] = {0, };
    double support[N_DRIVES] = { 0, };

    fprintf(o, "Transitions\n");
    fprintf(o, "-----------\n");
    for (l = layers, i = 0; i < n_layers; l++, i++) {
	fprintf(o, "z=%-8.2f h=%-4.2f density=%-4.2f %9s ", l->z, l->h, l->density, l->use_perimeter ? "perimeter" : "");
	for (t = &transitions[l->transition0], j = 0; j < l->n_transitions; t++, j++) {
	    if (j > 0) fprintf(o, "%41c", ' ');
	    fprintf(o, "t%d -> t%d ||| %7.2f -> %6.2f [ %6.2f || %6.2f ] %-6.2f%s", t->from, t->to, t->mm_from_runs, t->infill_mm, t->pre_mm, t->post_mm, t->support_mm, t->ping ? " ping" : "     ");
	    if (t->from != t->to) mm[t->from] += t->mm_from_runs;
	    waste[t->from] += t->pre_mm;
	    infill[t->from] += t->infill_mm;
	    waste[t->to] += t->post_mm;
	    support[t->to] += t->support_mm;
	    fprintf(o, "  ###  %12.2f | %-10.2f / %12.2f | %-10.2f", mm[t->from], waste[t->from], mm[t->to], waste[t->to]);
	    fprintf(o, " offset=%ld", t->offset);
	    fprintf(o, "\n");
	}
    }

    mm[transitions[n_transitions-1].to] += transition_final_mm;
    waste[transitions[n_transitions-1].to] += transition_final_waste;

    for (i = 0; i < N_DRIVES; i++) {
	if (mm[i] > 0) {
	    fprintf(o, "T%d: %7.2f, %6.2f wasted, %6.2f + %.2f saved\n", i, mm[i], waste[i], infill[i], support[i]);
	}
    }
}
