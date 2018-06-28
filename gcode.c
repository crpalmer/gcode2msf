#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "gcode.h"
#include "printer.h"
#include "transition-block.h"

typedef struct {
    enum {
	MOVE,
	START_TOWER,
	END_TOWER,
	SET_E,
	TOOL,
	PING,
	START,
	OTHER,
	DONE
    } t;
    union {
	struct {
	    double x, y, z, e;
	} move;
	struct {
	    int num, step;
	} ping;
	double e;
	int tool;
    } x;
    long pos;
} token_t;

extern printer_t *printer;

static FILE *f, *o;

static char buf[1024*1024];

int extrusions = 0;
int gcode_trace = 0;
int validate_only = 0;

static double last_x = 0, last_y = 0, last_z = 0, start_e = 0, last_e = 0, last_e_z = 0, acc_e = 0, last_reported_z = -1;
static bb_t bb;
static int tool = 0;
static int seen_tool = 0;
static int n_used_tools = 0;
static double tower_z = -1;
static int in_tower = 0;
static int seen_ping = 0;
static int started = 0;

run_t runs[MAX_RUNS];
int n_runs = 0;
int used_tool[N_DRIVES] = { 0, };

splice_t splices[MAX_RUNS];
int n_splices = 0;

static int
find_arg(const char *buf, char arg, double *val)
{
    size_t i;

    for (i = 0; buf[i]; i++) {
	if (buf[i] == ' ' && buf[i+1] == arg) {
	     if (sscanf(&buf[i+2], "%lf", val) > 0) {
		return 1;
	     }
	     return 0;
	}
    }
}

#define STRNCMP(a, b) strncmp(a, b, strlen(b))

static token_t
get_next_token_wrapped()
{
    token_t t;

    while (fgets(buf, sizeof(buf), f) != NULL) {
	t.pos = ftell(f);
	if (STRNCMP(buf, "G1 ") == 0) {
	    t.t = MOVE;
	    if (! find_arg(buf, 'X', &t.x.move.x)) t.x.move.x = last_x;
	    if (! find_arg(buf, 'Y', &t.x.move.y)) t.x.move.y = last_y;
	    if (! find_arg(buf, 'Z', &t.x.move.z)) t.x.move.z = last_z;
	    if (! find_arg(buf, 'E', &t.x.move.e)) t.x.move.e = last_e;
	    return t;
	}
	if (STRNCMP(buf, "; finishing tower layer") == 0 ||
	    STRNCMP(buf, "; finishing sparse tower layer") == 0) {
	    t.t = START_TOWER;
	    return t;
	}
	if (STRNCMP(buf, "; leaving transition tower") == 0) {
	    t.t = END_TOWER;
	    return t;
	}
	if (STRNCMP(buf, "; ping ") == 0) {
	    t.t = PING;
	    sscanf(buf, "; ping %d pause %d", &t.x.ping.num, &t.x.ping.step);
	    return t;
	}
	if (STRNCMP(buf, "G92 ") == 0) {
	    t.t = SET_E;
	    if (find_arg(buf, 'E', &t.x.e)) return t;
	}
	if (STRNCMP(buf, "; *** Main G-code ***") == 0 ||
	    STRNCMP(buf, "; layer 1, ") == 0) {
	    t.t = START;
	    return t;
	}
	if (buf[0] == 'T' && isdigit(buf[1])) {
	    t.t = TOOL;
	    t.x.tool = atoi(&buf[1]);
	    return t;
	}
	t.t = OTHER;
	return t;
    }

    t.t = DONE;
    return t;
}

static token_t
get_next_token()
{
    token_t t = get_next_token_wrapped();
    if (gcode_trace) {
	printf("%8ld ", t.pos);
	switch (t.t) {
	case MOVE: printf("MOVE (%f,%f,%f) e=%f\n", t.x.move.x, t.x.move.y, t.x.move.z, t.x.move.e); break;
	case START_TOWER: printf("START_TOWER\n"); break;
	case END_TOWER: printf("END_TOWER\n"); break;
	case PING: printf("PING %d.%d\n", t.x.ping.num, t.x.ping.step); break;
	case SET_E: printf("SET_E %f\n", t.x.e); break;
	case START: printf("START\n"); break;
	case TOOL: printf("TOOL %d\n", t.x.tool); break;
	case OTHER: break;
	default: printf("*** UNKNOWN TOKEN ****\n");
        }
    }
    return t;
}

static void
accumulate()
{
    acc_e += (last_e - start_e);
}

static void
reset_state()
{
    start_e = last_e;
    acc_e = 0;
    seen_ping = 0;
    bb_init(&bb);
}

static void
show_extrusion(char chr, int force)
{
    int bad = in_tower && tower_z != last_e_z && acc_e > 0;

    last_reported_z = last_e_z;
    if (bad || acc_e != 0 || force) {
	if (validate_only && ! bad) return;
	if (! validate_only && ! extrusions) return;

	printf("%c", chr);
	if (seen_tool) printf(" T%d", tool);
	printf(" Z %f", last_e_z);
	if (acc_e != 0) printf(" E %f", acc_e);
	if (bad) printf(" *********** z delta = %f", last_e_z - tower_z);
	if (seen_ping) printf(" [ping %d]", seen_ping);
	printf("\n");
    }
}

static void
add_run()
{
    if (acc_e != 0) {
	if (! used_tool[tool]) {
	    used_tool[tool] = 1;
	    n_used_tools++;
	}

	runs[n_runs].bb = bb;
	runs[n_runs].z = last_e_z;
	runs[n_runs].e = acc_e;
	runs[n_runs].t = tool;
	runs[n_runs].pre_transition = -1;
	runs[n_runs].post_transition = -1;
	n_runs++;
    }
}

static void
preprocess()
{
    reset_state();
    while (1) {
	token_t t = get_next_token();
	switch(t.t) {
	case MOVE:
	    bb_add_point(&bb, t.x.move.x, t.x.move.y);

	    if (t.x.move.e != last_e && t.x.move.z != last_e_z) {
		accumulate();
		if (started) {
		    add_run();
		    show_extrusion('+', 0);
		}
		last_e_z = t.x.move.z;
		reset_state();
	    }
	    last_e = t.x.move.e;
	    last_x = t.x.move.x;
	    last_y = t.x.move.y;
	    last_z = t.x.move.z;
	    break;
	case START_TOWER:
	    accumulate();
	    show_extrusion('+', 0);
	    reset_state();
	    tower_z = last_e_z;
	    in_tower = 1;
	    show_extrusion('>', 1);
	    break;
	case END_TOWER:
	    accumulate();
	    show_extrusion('<', 1);
	    in_tower = 0;
	    reset_state();
	    break;
	case SET_E:
	    accumulate();
	    start_e = t.x.e;
	    last_e = t.x.e;
	    break;
	case TOOL:
	    if (tool != t.x.tool) {
		if (! started) {
		    fprintf(stderr, "** ERROR *** Tool change before in prefix gcode\n");
		    exit(1);
		}
		accumulate();
		add_run();
		if (validate_only && acc_e == 0) printf("Z %.02f ******* Tool change with no extrusion, chroma may screw this up\n", last_z);
		show_extrusion('+', 0);
		reset_state();
		tool = t.x.tool;
		seen_tool = 1;
	    }
	    break;
	case PING:
	    seen_ping = t.x.ping.num;
	    break;
	case START:
	    started = 1;
	    accumulate();
	    reset_state();
	    break;
	case DONE:
	    return;
	case OTHER:
	    break;
	}
    }
}

static void
add_splice(int drive, double mm)
{
    splices[n_splices].drive = drive;
    splices[n_splices].mm = mm;
    n_splices++;
}

static void
generate_transition(transition_t *t, double *total_e)
{
    fprintf(o, "; Transition: %d->%d with %f mm\n", t->from, t->to, t->mm);
    (*total_e) += t->mm * printer->transition_target;
    if (t->from != t->to) add_splice(t->from, *total_e);
    (*total_e) += t->mm * (1 - printer->transition_target);	// this should be actual extruded when I really print the block
}

static void
produce_gcode()
{
    int l = 0;
    int t = 0;
    double last_e_z = 0, last_e_tool;
    int seen_tool = 0;
    int tool = 0;
    int started = 0;
    double total_e;

    last_e = last_x = last_y = last_z = 0;

    rewind(f);
    while (1) {
	token_t token = get_next_token();

	switch(token.t) {
	case MOVE: {
	    int may_need_transition = 0;

	    if (tool != last_e_tool) may_need_transition = 1;
	    if (token.x.move.z != last_e_z && layers[l].n_transitions == 1 && transitions[t].from == transitions[t].to) may_need_transition = 1;

	    if (t < n_transitions && started && token.x.move.e != last_e && may_need_transition) {
		last_e_z = token.x.move.z;
		last_e_tool = tool;
		generate_transition(&transitions[t], &total_e);
		assert(token.x.move.z == layers[l].z);
		t++;
		if (layers[l].transition0 + layers[l].n_transitions == t) l++;
		assert(layers[l].transition0 <= t && t < layers[l].transition0 + layers[l].n_transitions);
	    }
	    total_e += token.x.move.e - last_e;
	    last_e = token.x.move.e;
	    last_x = token.x.move.x;
	    last_y = token.x.move.y;
	    last_z = token.x.move.z;
	    fprintf(o, "%s", buf);
	    break;
	}
	case START:
	    started = 1;
	    break;
	case START_TOWER:
	case END_TOWER:
	case PING:
	case OTHER:
	    fprintf(o, "%s", buf);
	    break;
	case SET_E:
	    last_e = 0;
	    fprintf(o, "%s", buf);
	    break;
	case TOOL:
	    if (! seen_tool) {
		seen_tool = 1;
		last_e_tool = token.x.tool;
	    }
	    tool = token.x.tool;
	    fprintf(o, "; Switching to tool %d\n", tool);
	    break;
	case DONE:
	    add_splice(tool, total_e);
	    return;
	}
    }
}

void gcode_to_runs(const char *fname)
{
    if ((f = fopen(fname, "r")) == NULL) {
	perror(fname);
	return;
    }

    preprocess();
}

void gcode_to_msf_gcode(const char *output_fname)
{
    if ((o = fopen(output_fname, "w")) == NULL) {
	perror(output_fname);
	return;
    }

    produce_gcode();
    fclose(o);
    o = NULL;
printf("Made %d splices\n", n_splices);
}
