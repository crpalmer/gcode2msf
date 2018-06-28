#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "bb.h"
#include "gcode.h"
#include "materials.h"
#include "printer.h"

typedef struct {
    bb_t   bb;
    double z;
    double h;
    int	transition0;
    int n_transitions;
    double mm;
} layer_t;

typedef struct {
    int from, to;
    double mm;
} transition_t;

static int summary = 0;

static layer_t layers[MAX_RUNS];
static int n_layers;
static transition_t transitions[MAX_RUNS];
static int n_transitions = 0;
static bb_t model_bb;

struct {
    material_t *m;
    const char *colour;
} materials[N_DRIVES];

static printer_t *printer;

/* pre_transition is the amount of filament to consume for the transition prior to printing anything
 * post_transition is the amount of filament to consume for the transition after printing everything
 */

static transition_t *
get_pre_transition(int j)
{
    if (runs[j].pre_transition >= 0) return &transitions[runs[j].pre_transition];
    else return NULL;
}

static double
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

static double
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

static void
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

static void
output_summary()
{
    int i, j;
    int layer = 1;
    double last_layer_z = 0;
    double tool_mm[N_DRIVES] = { 0, };
    double tool_waste[N_DRIVES] = { 0, };

    printf("Layer by layer extrusions\n");
    printf("-------------------------\n");
    for (i = 0; i < n_runs; i = j) {
	printf("%5d %6.02f %6.02f", layer++, runs[i].z - last_layer_z, runs[i].z);
	last_layer_z = runs[i].z;
	for (j = i; j < n_runs && runs[i].z == runs[j].z; j++) {
	    printf(" %10.2f [%d]", runs[j].e, runs[j].t);
	}
	printf("\n");
    }
    printf("\n");
    printf("Extruder by extruder extrusions\n");
    printf("-------------------------------\n");
    for (i = 0; i < n_runs; i = j) {
	double mm = 0, waste = 0;
	for (j = i; j < n_runs && runs[i].t == runs[j].t; j++) {
	    mm += runs[j].e;
	    waste += get_pre_transition_mm(j) + get_post_transition_mm(j);
	}
	tool_mm[runs[i].t] += mm;
	tool_waste[runs[i].t] += waste;
	printf("T%d %10.4f mm %10.4f waste\n", runs[i].t, mm, waste);
    }
    for (i = 0; i < N_DRIVES; i++) {
	if (tool_mm[i] + tool_waste[i] != 0) printf("   TOTAL: T%d %10.2f mm %10.2f waste\n", i, tool_mm[i], tool_waste[i]);
    }
    printf("\n");
    double t_xy[2];
    transition_block_size(t_xy);
    printf("Transition block: area %f dimensions %fx%f model_bb=(%f,%f),(%f,%f)\n", transition_block_area(), t_xy[0], t_xy[1], BB_PRINTF_ARGS(&model_bb));
    printf("----------------\n");
    for (i = 0; i < n_layers; i++) {
	printf("z=%-6.2f height=%-4.2f mm=%-6.2f n-transitions=%d area=%f bb=(%f,%f),(%f,%f)\n", layers[i].z, layers[i].h, layers[i].mm, layers[i].n_transitions, transition_block_layer_area(i), BB_PRINTF_ARGS(&layers[i].bb));
    }
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
find_model_bb_to_tower_height()
{
    int i;

    bb_init(&model_bb);

    for (i = 0; i < n_layers; i++) {
	bb_add_bb(&model_bb, &layers[i].bb);
    }
}

static char *
float_to_hex(double f, char *buf)
{
    unsigned v;
    if (f == 0) strcpy(buf, "0");
    else {
	int shift = 0;

	if (f < 0) v = 1 << 31;
	else v = 0;
	f = fabs(f);

	while (f >= 2) {
	   f /= 2;
	   shift++;
	}
	while (f < 1) {
	   f *= 2;
	   shift--;
	}
	f = f - 1;
	v |= (unsigned) (f * (0x800000 + 0.5));
	v |= (shift + 0x7f) << 23;

	sprintf(buf, "%x", v);
    }

    return buf;
}

static void
produce_msf_colours(FILE *o)
{
    int i;
    int first = 1;

    fprintf(o, "cu:");
    for (i = 0; i < N_DRIVES; i++) {
	if (! first) fprintf(o, ";");
	first = 0;
	if (used_tool[i]) {
	    const char *c = materials[i].colour;
	    fprintf(o, "%d%s%s%s", materials[i].m->id+1, c ? c : "", c ? " " : "", materials[i].m->type);
	} else {
	    fprintf(o, "0");
	}
    }
    fprintf(o, "\n");
}

static void
count_or_produce_splices(FILE *o, int *n_out)
{
    int i, j, n = 0;
    char buf[20];
    double mm = 0;

    for (i = 0; i < n_runs; i = j) {
	for (j = i; j < n_runs && runs[i].t == runs[j].t; j++) {
	    mm += get_pre_transition_mm(j);
	    mm += runs[j].e;
	    mm += get_post_transition_mm(j);
	}
	if (o) fprintf(o, "(%02x,%s)\n", runs[i].t, float_to_hex(mm, buf));
	n++;
    }

    if (n_out) *n_out = n;
}

static void
produce_msf_splices(FILE *o)
{
    count_or_produce_splices(o, NULL);
}

static int
msf_n_splices()
{
    int n;

    count_or_produce_splices(NULL, &n);
    return n;
}

static void
produce_msf_splice_configuration(FILE *o, int id1, int id2)
{
    int dir;
    char buf1[20], buf2[20];

    if (! o) return;
    for (dir = 0; dir < (id1 == id2 ? 1 : 2); dir++) {
	int incoming = dir ? id1 : id2;
	int outgoing = dir ? id2 : id1;
	material_splice_t *splice = materials_find_splice(incoming, outgoing);
	fprintf(o, "(%d%d,%s,%s,%d)\n", incoming+1, outgoing+1, float_to_hex(splice->heat, buf1), float_to_hex(splice->compression, buf2), splice->reverse);
    }
}

static void
count_or_produce_splice_configurations(FILE *o, int *n_out)
{
    int i, j;
    int handled[N_DRIVES] = { 0, };
    int n;

    for (i = 0; i < N_DRIVES; i++) {
	if (! used_tool[i] || handled[materials[i].m->id]) continue;
	produce_msf_splice_configuration(o, materials[i].m->id, materials[i].m->id);
	n++;
	for (j = 0; j < N_DRIVES; j++) {
	    if (! used_tool[j] || handled[j]) continue;
	    if (materials[i].m->id == materials[j].m->id) handled[j] = 1;
	    else {
		produce_msf_splice_configuration(o, materials[i].m->id, materials[j].m->id);
		n++;
	    }
	}
    }
    if (n_out) *n_out = n;
}

static void
produce_msf_splice_configurations(FILE *o)
{
    count_or_produce_splice_configurations(o, NULL);
}

static int
msf_splice_configurations_n()
{
    int n;

    count_or_produce_splice_configurations(NULL, &n);
    return n;
}

static void
produce_msf(const char *fname)
{
    FILE *o = stdout;
    char buf[20];

    fprintf(o, "MSF1.4\n");
    produce_msf_colours(o);
    fprintf(o, "ppm:%s\n", float_to_hex(printer->pv / printer->calibration_len, buf));
    fprintf(o, "lo:%x\n", printer->loading_offset);
    fprintf(o, "ns:%x\n", msf_n_splices());
    fprintf(o, "np:0\n");
    fprintf(o, "nh:%04x\n", msf_splice_configurations_n());
    // TODO na:
    produce_msf_splices(o);
    // TODO pings
    produce_msf_splice_configurations(o);
}

static void
produce_gcode(const char *gcode)
{
}

static void process(const char *fname)
{
    gcode_to_runs(fname);
    compute_transition_tower();
    prune_transition_tower();
    find_model_bb_to_tower_height();
    produce_msf(fname);
    produce_gcode(fname);
    if (summary) output_summary();
}

static void
record_material(int i, const char *colour, const char *name)
{
    materials[i].m = materials_find(name);
    materials[i].colour = colour;
}

int main(int argc, char **argv)
{
    int i;

    for (i = 0; i < N_DRIVES; i++) {
	record_material(i, NULL, "Default PLA");
    }

    while (argc > 2) {
	    if (strcmp(argv[1], "--validate") == 0) validate_only = 1;
	    else if (strcmp(argv[1], "--summary") == 0) summary = 1;
	    else if (strcmp(argv[1], "--trace") == 0) gcode_trace = 1;
	    else if (strcmp(argv[1], "--extrusions") == 0) extrusions = 1;
	    else if (argc > 3 && argv[1][0] == '-' && isdigit(argv[1][1]) && argv[1][2] == '\0') {
		record_material(atoi(&argv[1][1])-1, argv[2], argv[3]);
		argc -= 2;
		argv += 2;
	    } else break;
	    argc--;
	    argv++;
    }

    if (! materials_load("materials.yml")) {
	perror("materials.yaml");
	exit(1);
    }

    if (argc != 3) {
	fprintf(stderr, "usage: [--validate | --summary | --trace | --extrusions | --[1-4] colour material] printer.yaml input.yaml\n");
	exit(0);
    }

    if ((printer = printer_load(argv[1])) == NULL) {
	perror(argv[1]);
    }

    fprintf(stderr, "Using printer: %s\n", printer->name);
    process(argv[2]);
}
