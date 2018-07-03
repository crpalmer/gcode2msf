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
const char *output_fname;

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
	    fprintf(o, "%d %s%s%s", m->m->id+1, c ? c : "", c ? " " : "", m->m->type);
	} else {
	    fprintf(o, "0");
	}
    }
    fprintf(o, ";\r\n");
}

static void
produce_msf_splices(FILE *o)
{
    int i;
    char buf[20];

    for (i = 0; i < n_splices; i++) {
	 fprintf(o, "(%02x,%s)\r\n", splices[i].drive, float_to_hex(splices[i].mm, buf));
    }
}

static void
produce_msf_pings(FILE *o)
{
    int i;
    char buf[20];

    for (i = 0; i < n_pings; i++) {
	fprintf(o, "(64,%s)\r\n", float_to_hex(pings[i].mm, buf));
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
	fprintf(o, "(%d%d,%s,%s,%d)\r\n", incoming+1, outgoing+1, float_to_hex(splice->heat, buf1), float_to_hex(splice->compression, buf2), splice->reverse);
    }
}

static void
count_or_produce_splice_configurations(FILE *o, int *n_out)
{
    int i, j;
    int handled[N_DRIVES] = { 0, };
    int n = 0;

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

    fprintf(o, "MSF1.4\r\n");
    produce_msf_colours(o);
    fprintf(o, "ppm:%s\r\n", float_to_hex(printer->pv / printer->calibration_len, buf));
    fprintf(o, "lo:%04x\r\n", printer->loading_offset);
    fprintf(o, "ns:%04x\r\n", n_splices);
    fprintf(o, "np:%04x\r\n", n_pings);
    fprintf(o, "nh:0000\r\n");
    fprintf(o, "na:%04x\r\n", msf_splice_configurations_n());
    // TODO na:
    produce_msf_splices(o);
    produce_msf_pings(o);
    produce_msf_splice_configurations(o);
}

static void
output_material_usage_and_transition_block()
{
    double used[N_DRIVES] = {0, };
    double waste[N_DRIVES] = { 0, };
    double last = 0;
    int i;

    fprintf(stderr, "transition block:  (%.2f, %.2f) x (%.2f, %.2f)\n", transition_block.x, transition_block.y, transition_block.w, transition_block.h);
    fprintf(stderr, "transition layers: %d\n", n_transitions);
    fprintf(stderr, "number of splices: %d\n", n_splices);
    fprintf(stderr, "number of pings:   %d\n", n_pings);

    for (i = 0; i < n_splices; i++) {
	used[splices[i].drive] += splices[i].mm - last;
	waste[splices[i].drive] += splices[i].waste;
	last = splices[i].mm;
    }

    fprintf(stderr, "\nFilament usage:\n");
    for (i = 0; i < N_DRIVES; i++) {
	if (used[i]) printf("T%d: %9.2f mm + %9.2f mm waste => %9.2f mm (%.3f m)\n", i, used[i]-waste[i], waste[i], used[i], used[i]/1000);
    }
}

static int
ends_with(const char *str, const char *suffix)
{
    int len_str = strlen(str);
    int len_suffix = strlen(suffix);

    return len_str > len_suffix && strcmp(&str[len_str - len_suffix], suffix) == 0;
}

static char *
get_msf_fname(const char *base)
{
    char *fname;

    fname = malloc(strlen(base) + 20);
    strcpy(fname, base);
    if (ends_with(fname, ".gcode")) fname[strlen(fname)-6] = '\0';
    if (! ends_with(fname, ".msf")) strcat(fname, ".msf");
    return fname;
}

static void process(const char *fname)
{
    char *msf_fname;
    char *gcode_fname;

    msf_fname = get_msf_fname(output_fname ? output_fname : fname);
    gcode_fname = malloc(strlen(msf_fname) + 20);
    sprintf(gcode_fname, "%s.gcode", msf_fname);

    gcode_to_runs(fname);
    transition_block_create_from_runs();
    printf("Outputting to %s\n", msf_fname);

    gcode_to_msf_gcode(gcode_fname);
    produce_msf(msf_fname);
    if (summary) output_summary();
    else output_material_usage_and_transition_block();
}

int main(int argc, char **argv)
{
    init_active_materials();

    while (argc > 2) {
	    if (strcmp(argv[1], "--validate") == 0) validate_only = 1;
	    else if (strcmp(argv[1], "--summary") == 0) summary = 1;
	    else if (strcmp(argv[1], "--trace") == 0) gcode_trace = 1;
	    else if (strcmp(argv[1], "--extrusions") == 0) extrusions = 1;
	    else if (strcmp(argv[1], "--reduce-pings") == 0) reduce_pings = 1;
	    else if (strcmp(argv[1], "--output") == 0 || strcmp(argv[1], "-o") == 0) {
		output_fname = argv[2];
		argc--;
		argv++;
	    } else if (argc > 2 && argv[1][0] == '-' && argv[1][1] == 'c' && isdigit(argv[1][2]) && argv[1][3] == '\0') {
		set_active_material(atoi(&argv[1][2])-1, NULL, argv[2], UNKNOWN);
		argc--;
		argv++;
	    } else if (argc > 2 && argv[1][0] == '-' && argv[1][1] == 'm' && isdigit(argv[1][2]) && argv[1][3] == '\0') {
		set_active_material(atoi(&argv[1][2])-1, argv[2], NULL, UNKNOWN);
		argc--;
		argv++;
	    } else if (argc > 2 && argv[1][0] == '-' && argv[1][1] == 's' && isdigit(argv[1][2]) && argv[1][3] == '\0') {
		colour_strength_t s;

		if (strcasecmp(argv[2], "weak") == 0) s = WEAK;
		else if (strcasecmp(argv[2], "medium") == 0) s = MEDIUM;
		else if (strcasecmp(argv[2], "strong") == 0) s = STRONG;
		else {
		    fprintf(stderr, "Invalid colour strength: %s, valid values of WEAK, MEDIUM or STRONG\n", argv[2]);
		    goto usage;
		}

		set_active_material(atoi(&argv[1][2])-1, NULL, NULL, s);
		argc--;
		argv++;
	    } else if (argv[1][0] == '-') {
		fprintf(stderr, "unknown argument: %s\n", argv[1]);
usage:
		fprintf(stderr, "usage: [<flags> | <colour> | <material> | <strength> | --output fname] printer.yml gcode.gcode\n");
		fprintf(stderr, "  <colour>:   -cX colour to set the colour of drive \"X\" to \"colour\"\n");
		fprintf(stderr, "  <material>: -mX material to set the material of drive \"X\" to \"material\"\n");
		fprintf(stderr, "  <strength>: -sX strength to set the strength of the material's colour (WEAK, MEDIUM or STRONG)\n");
		fprintf(stderr, "  <flags>: any number of:\n");
		fprintf(stderr, "           --trace:        trace the gcode as it is processed\n");
		fprintf(stderr, "           --summary:      provide a more detailed summary of the print\n");
		fprintf(stderr, "           --reduce-pings: ping less frequently as the print gets longer and longer\n");
		exit(0);
	    } else {
		break;
	    }
	    argc--;
	    argv++;
    }

    if (argc != 3) goto usage;

    if (! materials_load("materials.yml")) {
	perror("materials.yaml");
	exit(1);
    }

    if (! printer_load(argv[1])) {
	perror(argv[1]);
    }

    fprintf(stderr, "Using printer: %s\n", printer->name);
    process(argv[2]);
}
