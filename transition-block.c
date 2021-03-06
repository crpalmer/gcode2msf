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

#define EXTRA_FILAMENT  150

layer_t layers[MAX_RUNS];
int n_layers;
double layer_mm[MAX_RUNS];
transition_t transitions[MAX_RUNS];
int n_transitions = 0;
transition_block_t transition_block;
int reduce_pings = 0;
double transition_final_mm;
double transition_final_waste;
prime_info_t prime_info;

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

int
transition_block_size(double xy[2], int strategy)
{
    double area = transition_block_area();
    double sqrt_area = sqrt(area);

    switch (strategy) {
    case 0: {		// golden ratio
	double ratio = sqrt((1 + sqrt(5))/2);	// Golden ratio
	xy[0] = sqrt_area / ratio;
	xy[1] = sqrt_area * ratio;
	break;
    }
    case 1:		// square
	xy[0] = sqrt_area;
	xy[1] = sqrt_area;
	break;
    case 2:		// wide
	xy[0] = sqrt_area * 2;
	xy[1] = area / xy[0];
	break;
    case 3:		// tall
	xy[1] = sqrt_area * 2;
	xy[0] = area / xy[1];
	break;
    case 4:		// extra wide
	xy[0] = sqrt_area * 3;
	xy[1] = area / xy[0];
	break;
    case 5:		// extra tall
	xy[1] = sqrt_area * 3;
	xy[0] = area / xy[1];
	break;
    default:
	return 0;
    }

    assert(fabs(xy[0] * xy[1] - area) < 0.0001);
    return 1;
}

static double
transition_length(int from, int to, double total_mm)
{
    double mn = printer->min_transition_len;
    double mx = total_mm < printer->ping_stabilize_mm ? printer->early_transition_len : printer->transition_len;
    active_material_t *in = get_active_material(to);
    active_material_t *out = get_active_material(from);
    double factor;

    if (from == to) return 0;

    if (total_mm < printer->ping_stabilize_mm) return printer->early_transition_len;

    if (in->strength == STRONG) {
	if (out->strength == STRONG) factor = 0.5;
	else if (out->strength == WEAK) factor = 0;
	else factor = 0.25;
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

#define EARLY_PING_MM		2000
#define EARLY_PING_THRESHOLD	350
#define PING_THRESHOLD		425
#define PING_SECOND_THRESHOLD	(PING_THRESHOLD*2)
#define PING_SECOND_E		10000
#define PING_THIRD_THRESHOLD	(PING_SECOND_THRESHOLD*4)
#define PING_THIRD_E		50000

static double
get_ping_threshold(double total_mm)
{
    if (total_mm < EARLY_PING_MM) return EARLY_PING_THRESHOLD;

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
    t->next_move_no_extrusion = pre_run->next_move_no_extrusion;
    t->needs_retraction = ! pre_run->ends_with_retraction;

    if (printer->transition_in_infill && pre_run->trailing_infill_mm > 0) {
	if (0 && get_active_material(from)->strength == WEAK && get_active_material(to)->strength == STRONG) {
	    t->avail_infill = 0;
	} else {
	    t->avail_infill = pre_run->trailing_infill_mm;
	}
    } else {
	t->avail_infill = 0;
    }
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
    transition_final_waste += 0.01 * transition_final_mm;
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
place_transition_block()
{
    double x, y;
    double z = layers[n_layers-1].z;
    int strategy = 0;
    double size[2];

    while (transition_block_size(size, strategy++)) {
	if (size[0] > printer->nozzle*4 && size[1] > printer->nozzle*4 &&
	    bed_usage_place_object(bed_usage, size[0], size[1], z, &x, &y)) {
	    transition_block.x = x;
	    transition_block.y = y;
	    transition_block.w = size[0];
	    transition_block.h = size[1];
	    transition_block.area = size[0] * size[1];
	    return;
	}
	fprintf(stderr, "Failed to place transition block %fx%f.  Aborting.\n", size[0], size[1]);
    }

    bed_usage_print(bed_usage, stderr);
    fprintf(stderr, "Failed to place transition block.  Aborting.\n");
    exit(1);
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

#define MAX_PRIME_LINES	20

static void
place_prime()
{
    int n;
    double mm3 = filament_length_to_mm3(printer->prime_mm);
    double len = mm3 / printer->nozzle / layers[0].h;

    for (n = 1; n <= MAX_PRIME_LINES; n++) {
	double w = len / n;
	double h = n * printer->nozzle;
	double x, y;

	if (bed_usage_place_and_add_object(bed_usage, w, h, -1, 'P', &x, &y)) {
	    prime_info.len = w;
	    prime_info.n   = n;
	    prime_info.x   = x;
	    prime_info.y   = y;
	    prime_info.e   = filament_mm3_to_length(w * printer->nozzle * layers[0].h);
	    printf("Placed priming at %f,%f with %d lines of length %f\n", x, y, n, w);
	    return;
	}
    }

    fprintf(stderr, "WARNING: failed to place purge lines\n");
}

void
transition_block_create_from_runs()
{
    int iterations = 0;
    compute_transition_tower();
    prune_transition_tower();
    if (n_transitions > 0) {
	do {
	    iterations++;
	    place_transition_block();
	} while (! fix_constraints());
	bed_usage_add_object(bed_usage, transition_block.x, transition_block.y, transition_block.w, transition_block.h, 'T');
	printf("It took %d iterations to stabilize the block\n", iterations);
    } else {
	transition_final_waste = 0;
    }
    if (printer->prime_mm > 0) place_prime();
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
	    if (t->next_move_no_extrusion) fprintf(o, " no-extrusion");
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
