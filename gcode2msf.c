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
#include "transition-block.h"

static int summary = 0;

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
    printf("Transition block: area %f transition_block=(%f,%f)x(%f,%f)\n", t_xy[0] * t_xy[1], transition_block.x, transition_block.y, transition_block.w, transition_block.h);
    printf("----------------\n");
    for (i = 0; i < n_layers; i++) {
	printf("z=%-6.2f height=%-4.2f mm=%-6.2f n-transitions=%d bb=(%f,%f),(%f,%f)\n", layers[i].z, layers[i].h, layers[i].mm, layers[i].n_transitions, BB_PRINTF_ARGS(&layers[i].bb));
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
	    active_material_t *m = get_active_material(i);
	    const char *c = m->colour;
	    fprintf(o, "%d%s%s%s", m->m->id+1, c ? c : "", c ? " " : "", m->m->type);
	} else {
	    fprintf(o, "0");
	}
    }
    fprintf(o, "\n");
}

static void
produce_msf_splices(FILE *o)
{
    int i;
    char buf[20];

    for (i = 0; i < n_splices; i++) {
	 fprintf(o, "(%02x,%s)\n", splices[i].drive, float_to_hex(splices[i].mm, buf));
    }
}

static void
produce_msf_pings(FILE *o)
{
    int i;
    char buf[20];

    for (i = 0; i < n_pings; i++) {
	fprintf(o, "(64,%s)\n", float_to_hex(pings[i].mm, buf));
    }
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
	active_material_t *m = get_active_material(i);
	if (! used_tool[i] || handled[m->m->id]) continue;
	produce_msf_splice_configuration(o, m->m->id, m->m->id);
	n++;
	for (j = 0; j < N_DRIVES; j++) {
	    active_material_t *m2 = get_active_material(j);

	    if (! used_tool[j] || handled[j]) continue;
	    if (m->m->id == m2->m->id) handled[j] = 1;
	    else {
		produce_msf_splice_configuration(o, m->m->id, m2->m->id);
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
    FILE *o;
    char buf[20];

    if ((o = fopen(fname, "w")) == NULL) {
	perror(fname);
	return;
    }

    fprintf(o, "MSF1.4\n");
    produce_msf_colours(o);
    fprintf(o, "ppm:%s\n", float_to_hex(printer->pv / printer->calibration_len, buf));
    fprintf(o, "lo:%x\n", printer->loading_offset);
    fprintf(o, "ns:%x\n", n_splices);
    fprintf(o, "np:%04x\n", n_pings);
    fprintf(o, "nh:%04x\n", msf_splice_configurations_n());
    // TODO na:
    produce_msf_splices(o);
    produce_msf_pings(o);
    produce_msf_splice_configurations(o);
}

static void process(const char *fname)
{
    gcode_to_runs(fname);
    transition_block_create_from_runs();
    gcode_to_msf_gcode("/tmp/gcode");
    produce_msf("/tmp/msf");
    if (summary) output_summary();
}

int main(int argc, char **argv)
{
    while (argc > 2) {
	    if (strcmp(argv[1], "--validate") == 0) validate_only = 1;
	    else if (strcmp(argv[1], "--summary") == 0) summary = 1;
	    else if (strcmp(argv[1], "--trace") == 0) gcode_trace = 1;
	    else if (strcmp(argv[1], "--extrusions") == 0) extrusions = 1;
	    else if (argc > 3 && argv[1][0] == '-' && isdigit(argv[1][1]) && argv[1][2] == '\0') {
		set_active_material(atoi(&argv[1][1])-1, argv[3], argv[2], UNKNOWN);
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

    if (! printer_load(argv[1])) {
	perror(argv[1]);
    }

    fprintf(stderr, "Using printer: %s\n", printer->name);
    process(argv[2]);
}
